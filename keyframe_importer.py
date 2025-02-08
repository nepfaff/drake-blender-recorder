# SPDX-License-Identifier: BSD-2-Clause

import pickle

import bpy

from bpy.props import StringProperty
from bpy.types import Operator, Panel
from bpy_extras.io_utils import ImportHelper

bl_info = {
    "name": "Keyframe Importer",
    "author": "Nicholas Pfaff",
    "version": (1, 0),
    "blender": (4, 0, 0),
    "location": "View3D > UI > Keyframe Importer",
    "description": "Import keyframes from recording server",
    "category": "Animation",
}


class KeyframeImportOperator(Operator, ImportHelper):
    """
    Import keyframes from a pickle file.

    The data must have the following format:

    ```
    [
        [
            {
                "name": "object_name",
                "location": (x, y, z),
                "rotation_quaternion": (x, y, z, w),
            },
            ...
        ]
    ]
    ```
    where the outer list is a list of frames and the inner list is a list of objects.

    The keyframes are only added for the objects that are already present in the scene.
    Hence, the recommended workflow is to first load the .blend file that was exported
    by `drake_recording_server.py` and then import the keyframe pickle file that was
    saved by the server.
    """

    bl_idname = "animation.import_keyframes"
    bl_label = "Import Keyframes"

    filename_ext = ".pkl"
    filter_glob: StringProperty(default="*.pkl", options={"HIDDEN"})

    def execute(self, context):
        try:
            with open(self.filepath, "rb") as f:
                keyframes = pickle.load(f)

            for frame_idx, frame_data in enumerate(keyframes):
                # Set the current frame.
                bpy.context.scene.frame_set(frame_idx)

                for obj_data in frame_data:
                    obj_name = obj_data["name"]
                    if obj_name not in bpy.data.objects:
                        self.report(
                            {"WARNING"},
                            f"Object {obj_name} not found in scene",
                        )
                        continue

                    obj = bpy.data.objects[obj_name]

                    # Set location.
                    obj.location = obj_data["location"]
                    obj.keyframe_insert(data_path="location")

                    # Set rotation.
                    obj.rotation_mode = "QUATERNION"
                    obj.rotation_quaternion = obj_data["rotation_quaternion"]
                    obj.keyframe_insert(data_path="rotation_quaternion")

            # Set animation range.
            bpy.context.scene.frame_start = 0
            bpy.context.scene.frame_end = len(keyframes) - 1

            self.report({"INFO"}, f"Successfully imported {len(keyframes)} frames")
            return {"FINISHED"}

        except Exception as e:
            self.report({"ERROR"}, f"Error importing keyframes: {str(e)}")
            return {"CANCELLED"}


class KeyframeImporterPanel(Panel):
    """Creates a Panel in the 3D Viewport UI"""

    bl_label = "Keyframe Importer"
    bl_idname = "VIEW3D_PT_keyframe_importer"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Keyframe Importer"

    def draw(self, context):
        layout = self.layout
        layout.operator(KeyframeImportOperator.bl_idname, text="Import Keyframes")


def register():
    bpy.utils.register_class(KeyframeImportOperator)
    bpy.utils.register_class(KeyframeImporterPanel)


def unregister():
    bpy.utils.unregister_class(KeyframeImportOperator)
    bpy.utils.unregister_class(KeyframeImporterPanel)


if __name__ == "__main__":
    register()
