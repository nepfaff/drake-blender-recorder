# SPDX-License-Identifier: BSD-2-Clause
# Modified from https://github.com/RobotLocomotion/drake-blender/blob/main/server.py

import argparse
import dataclasses as dc
import datetime
import io
import math
import pickle
import tempfile
import typing

from pathlib import Path
from types import NoneType

import bpy
import flask

from PIL import Image


@dc.dataclass
class RenderParams:
    """A dataclass that encapsulates all the necessary parameters to render a color,
    depth, or label image.

    https://drake.mit.edu/doxygen_cxx/group__render__engine__gltf__client__server__api.html#render-endpoint-form-data
    """

    scene: Path
    """The glTF input file."""

    scene_sha256: str
    """The checksum of `scene`."""

    image_type: typing.Literal["color", "depth", "label"]
    """The type of image being rendered."""

    width: int
    """Width of the desired rendered image in pixels."""

    height: int
    """Height of the desired rendered image in pixels."""

    near: float
    """The near clipping plane of the camera as specified by the
    RenderCameraCore's ClippingRange::near() value."""

    far: float
    """The far clipping plane of the camera as specified by the
    RenderCameraCore's ClippingRange::far() value."""

    focal_x: float
    """The focal length x, in pixels, as specified by the
    systems::sensors::CameraInfo::focal_x() value."""

    focal_y: float
    """The focal length y, in pixels, as specified by the
    systems::sensors::CameraInfo::focal_y() value."""

    fov_x: float
    """The field of view in the x-direction (in radians) as specified by the
    systems::sensors::CameraInfo::fov_x() value."""

    fov_y: float
    """The field of view in the y-direction (in radians) as specified by the
    systems::sensors::CameraInfo::fov_y() value."""

    center_x: float
    """The principal point's x coordinate in pixels as specified by the
    systems::sensors::CameraInfo::center_x() value."""

    center_y: float
    """The principal point's y coordinate in pixels as specified by the
    systems::sensors::CameraInfo::center_y() value."""

    min_depth: typing.Optional[float] = None
    """The minimum depth range as specified by a depth sensor's
    DepthRange::min_depth(). Only provided when image_type="depth"."""

    max_depth: typing.Optional[float] = None
    """The maximum depth range as specified by a depth sensor's
    DepthRange::max_depth(). Only provided when image_type="depth"."""


class Blender:
    """Encapsulates our access to blender.

    Note that even though this is a class, bpy is a singleton so likewise you
    should only ever create one instance of this class.
    """

    def __init__(
        self,
        *,
        blend_file: Path = None,
        bpy_settings_file: Path = None,
        export_path: Path = None,
        keyframe_dump_path: Path = None,
    ):
        self._blend_file = blend_file
        self._bpy_settings_file = bpy_settings_file
        self._export_path = export_path
        self._keyframe_dump_path = keyframe_dump_path

        self._keyframes = []

        if self._keyframe_dump_path.exists():
            raise ValueError(
                f"Keyframe dump path {self._keyframe_dump_path} already exists."
            )

    def reset_scene(self):
        """
        Resets the scene in Blender by loading the default startup file, and
        then removes the default cube object.
        """
        bpy.ops.wm.read_factory_settings()
        for item in bpy.data.objects:
            item.select_set(True)
        bpy.ops.object.delete()

    def save_keyframe(self, *, params: RenderParams):
        """
        Saves the current object poses as a keyframe.
        """
        # Load the blend file to set up the basic scene if provided.
        if self._blend_file is not None:
            bpy.ops.wm.open_mainfile(filepath=str(self._blend_file))
        else:
            self.reset_scene()

        # Apply the user's custom settings.
        if self._bpy_settings_file:
            with open(self._bpy_settings_file) as f:
                code = compile(f.read(), self._bpy_settings_file, "exec")
                exec(code, {"bpy": bpy}, dict())

        self._client_objects = bpy.data.collections.new("ClientObjects")
        old_count = len(bpy.data.objects)
        # Import a glTF file. Note that the Blender glTF importer imposes a
        # +90 degree rotation around the X-axis when loading meshes. Thus, we
        # counterbalance the rotation right after the glTF-loading.
        bpy.ops.import_scene.gltf(filepath=str(params.scene))
        new_count = len(bpy.data.objects)
        # Reality check that all of the imported objects are selected by
        # default.
        assert new_count - old_count == len(bpy.context.selected_objects)

        # TODO(#39) This rotation is very suspicious. Get to the bottom of it.
        # We explicitly specify the pivot point for the rotation to allow for
        # glTF files with root nodes with arbitrary positioning. We simply want
        # to rotate around the world origin.
        bpy.ops.transform.rotate(
            value=math.pi / 2,
            orient_axis="X",
            orient_type="GLOBAL",
            center_override=(0, 0, 0),
        )

        # Store the poses of the newly imported objects.
        imported_objects = bpy.context.selected_objects
        frame_data = []
        for obj in imported_objects:
            pose_data = {
                "name": obj.name,
                "location": list(obj.location),
                "rotation_quaternion": list(obj.rotation_quaternion),
            }
            frame_data.append(pose_data)
        self._keyframes.append(frame_data)

        # Create a new collection for imported objects and move them there.
        drake_objects = bpy.data.collections.new("DrakeObjects")
        bpy.context.scene.collection.children.link(drake_objects)
        for obj in bpy.context.selected_objects:
            # Unlink from current collections.
            for coll in obj.users_collection:
                coll.objects.unlink(obj)
            # Link to our collection.
            drake_objects.objects.link(obj)

        # Export the first scene.
        if self._export_path is not None and len(self._keyframes) == 1:
            self._export_path.parent.mkdir(parents=True, exist_ok=True)
            bpy.ops.wm.save_as_mainfile(filepath=str(self._export_path))

    def dump_keyframes_to_disk(self):
        with open(self._keyframe_dump_path, "wb") as f:
            pickle.dump(self._keyframes, f)


class ServerApp(flask.Flask):
    """
    The long-running Flask server application.

    This class is used for recording Drake simulation states as a list of object poses
    that can then be imported as keyframes in Blender.

    Specifically, the server implements Drake's glTF Render Client-Server API but
    instead of rendering an image, it saves the object poses as keyframes in Blender.
    """

    def __init__(
        self,
        *,
        temp_dir,
        blend_file: Path = None,
        bpy_settings_file: Path = None,
        export_path: Path = None,
        keyframe_dump_path: Path = None,
    ):
        super().__init__("drake_blender_recording_server")

        self._temp_dir = temp_dir
        self._blender = Blender(
            blend_file=blend_file,
            bpy_settings_file=bpy_settings_file,
            export_path=export_path,
            keyframe_dump_path=keyframe_dump_path,
        )

        self.add_url_rule("/", view_func=self._root_endpoint)

        endpoint = "/render"
        self.add_url_rule(
            rule=endpoint,
            endpoint=endpoint,
            methods=["POST"],
            view_func=self._render_endpoint,
        )

    def _root_endpoint(self):
        """Displays a banner page at the server root."""
        return """\
        <!doctype html>
        <html><body><h1>Drake Blender Recording Server</h1></body></html>
        """

    def _render_endpoint(self):
        """
        Accepts a request to render and returns the generated image.

        NOTE that in practice this endpoint is saving the object poses and returns a
        fake image to satisfy the caller.
        """
        try:
            params = self._parse_params(flask.request)
            buffer = self._save_keyframe(params)

            # Create the fake image.
            img = Image.new("RGB", (params.width, params.height), color="black")
            buffer = io.BytesIO()
            img.save(buffer, format="PNG")
            buffer.seek(0)

            return flask.send_file(buffer, mimetype="image/png")
        except Exception as e:
            code = 500
            message = f"Internal server error: {repr(e)}"
            return (
                {
                    "error": True,
                    "message": message,
                    "code": code,
                },
                code,
            )

    def _parse_params(self, request: flask.Request) -> RenderParams:
        """Converts an http request to a RenderParams."""
        result = dict()

        # Compute a lookup table for known form field names.
        param_fields = {x.name: x for x in dc.fields(RenderParams)}
        del param_fields["scene"]

        # Copy all of the form data into the result.
        for name, value in request.form.items():
            if name == "submit":
                # Ignore the html boilerplate.
                continue
            field = param_fields[name]
            type_origin = typing.get_origin(field.type)
            type_args = typing.get_args(field.type)
            if field.type in (int, float, str):
                result[name] = field.type(value)
            elif type_origin == typing.Literal:
                if value not in type_args:
                    raise ValueError(f"Invalid literal for {name}")
                result[name] = value
            elif type_origin == typing.Union:
                # In our dataclass we declare a typing.Optional but that's just
                # sugar for typing.Union[T, typing.NoneType]. Here, we need to
                # parse the typing.Union spelling; we can assume the only use
                # of Union is for an Optional.
                assert len(type_args) == 2
                assert type_args[1] == NoneType
                result[name] = type_args[0](value)
            else:
                raise NotImplementedError(name)

        # Save the glTF scene data. Note that we don't check the scene_sha256
        # checksum; it seems unlikely that it could ever fail without flask
        # detecting the error. In any case, the blender glTF loader should
        # reject malformed files; we don't need to fail-fast.
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        scene = Path(f"{self._temp_dir}/{timestamp}.gltf")
        assert len(request.files) == 1
        request.files["scene"].save(scene)
        result["scene"] = scene

        return RenderParams(**result)

    def _save_keyframe(self, params: RenderParams) -> None:
        """
        Saves the current object poses as a keyframe and dump all poses so far to disk.
        """
        self._blender.save_keyframe(params=params)
        print(f"Saved keyframe {len(self._blender._keyframes)}")

        # Clean up the temporary glTF file.
        params.scene.unlink(missing_ok=True)

        self._blender.dump_keyframes_to_disk()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        type=str,
        default="127.0.0.1",
        help="URL to host on, default: %(default)s.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port to host on, default: %(default)s.",
    )
    parser.add_argument(
        "--blend_file",
        type=Path,
        metavar="FILE",
        help="Path to a *.blend file.",
    )
    parser.add_argument(
        "--export_path",
        required=True,
        type=Path,
        help="Path to export the Blender scene before each render.",
    )
    parser.add_argument(
        "--keyframe_dump_path",
        required=True,
        type=Path,
        help="Path to dump keyframes to disk. Must be a pickle file.",
    )
    parser.add_argument(
        "--bpy_settings_file",
        type=Path,
        metavar="FILE",
        help="Path to a *.py file that the server will exec() to configure "
        "blender. For example, the settings file might contain the line "
        '`bpy.context.scene.render.engine = "EEVEE"` (with no backticks). '
        "The settings file will be applied after loading the --blend_file "
        "(if any) so that it has priority.",
    )
    args = parser.parse_args()

    if args.export_path.suffix != ".blend":
        raise ValueError(
            "Expected export_path to have '.blend' suffix, "
            f"got '{args.export_path.suffix}'"
        )
    if args.keyframe_dump_path.suffix != ".pkl":
        raise ValueError(
            "Expected keyframe_dump_path to have '.pkl' suffix, "
            f"got '{args.keyframe_dump_path.suffix}'"
        )

    prefix = "drake_blender_recorder_"
    with tempfile.TemporaryDirectory(prefix=prefix) as temp_dir:
        app = ServerApp(
            temp_dir=temp_dir,
            blend_file=args.blend_file,
            bpy_settings_file=args.bpy_settings_file,
            export_path=args.export_path,
            keyframe_dump_path=args.keyframe_dump_path,
        )
        app.run(host=args.host, port=args.port, threaded=False)


if __name__ == "__main__":
    main()
