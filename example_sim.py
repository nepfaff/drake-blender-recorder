# SPDX-License-Identifier: MIT-0

import numpy as np

from manipulation.meshcat_utils import WsgButton
from manipulation.station import (
    LoadScenario,
    MakeHardwareStation,
    MakeMultibodyPlant,
    RobotDiagram,
)
from manipulation.systems import AddIiwaDifferentialIK, MultibodyPositionToBodyPose
from pydrake.all import (
    ApplySimulatorConfig,
    Context,
    DiagramBuilder,
    MeshcatPoseSliders,
    Simulator,
    StartMeshcat,
    VideoWriter,
)
from tqdm import tqdm

scenario_str = """
# How long to run the simulation for.
simulation_duration: 10.0

# The models to add to the simulation.
directives:
- add_directives:
    file: package://manipulation/iiwa_and_wsg.dmd.yaml
- add_directives:
    file: package://manipulation/two_bins_w_cameras.dmd.yaml
- add_model:
    name: mustard
    file: package://manipulation/hydro/006_mustard_bottle.sdf
    default_free_body_pose:
        base_link_mustard:
            translation: [0.55, 0.1, 0]
            rotation: !Rpy { deg: [0, 0, 45]}

model_drivers:
    iiwa: !IiwaDriver
      control_mode: position_only
      hand_model_name: wsg
    wsg: !SchunkWsgDriver {}

cameras:
    # This camera will trigger the keyframe saving callback in the recording server.
    recording_camera:
        name: recording_camera
        renderer_name: blender
        renderer_class: !RenderEngineGltfClientParams
            base_url: http://127.0.0.1:8000
        width: 8
        height: 8
        # How many frames per second to record.
        fps: 24
"""


class _ProgressBar:
    def __init__(self, simulation_duration: float):
        self._tqdm = tqdm(total=simulation_duration)
        self._current_time = 0.0

    def __call__(self, context: Context):
        old_time = self._current_time
        self._current_time = context.get_time()
        self._tqdm.update(self._current_time - old_time)


def main():
    scenario = LoadScenario(data=scenario_str)

    def prebuild_callback(builder: DiagramBuilder):
        # Connect a video writer for querying the camera at a given frame rate.
        camera = scenario.cameras["recording_camera"]
        writer = VideoWriter(
            filename=f"{camera.name}.mp4",
            fps=camera.fps,
            backend="cv2",
        )
        sensor = builder.GetSubsystemByName(f"rgbd_sensor_{camera.name}")
        builder.AddSystem(writer)
        writer.ConnectRgbdSensor(builder=builder, sensor=sensor)

    # Create the scene.
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    station: RobotDiagram = builder.AddSystem(
        MakeHardwareStation(
            scenario=scenario, meshcat=meshcat, prebuild_callback=prebuild_callback
        )
    )

    # Set up the differential IK controller.
    controller_plant = MakeMultibodyPlant(
        scenario=scenario, model_instance_names=["iiwa"]
    )
    differential_ik = AddIiwaDifferentialIK(
        builder=builder,
        plant=controller_plant,
        frame=controller_plant.GetFrameByName("iiwa_link_7"),
    )
    builder.Connect(
        differential_ik.get_output_port(),
        station.GetInputPort("iiwa.position"),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.state_estimated"),
        differential_ik.GetInputPort("robot_state"),
    )

    # Set up teleop widgets
    teleop = builder.AddSystem(
        MeshcatPoseSliders(
            meshcat,
            lower_limit=[0, -0.5, -np.pi, -0.6, -0.8, 0.0],
            upper_limit=[2 * np.pi, np.pi, np.pi, 0.8, 0.3, 1.1],
        )
    )
    builder.Connect(
        teleop.get_output_port(), differential_ik.GetInputPort("X_WE_desired")
    )
    ee_pose = builder.AddSystem(
        MultibodyPositionToBodyPose(
            plant=controller_plant, body=controller_plant.GetBodyByName("iiwa_link_7")
        )
    )
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"), ee_pose.get_input_port()
    )
    builder.Connect(ee_pose.get_output_port(), teleop.get_input_port())
    wsg_teleop = builder.AddSystem(WsgButton(meshcat))
    builder.Connect(wsg_teleop.get_output_port(0), station.GetInputPort("wsg.position"))

    diagram = builder.Build()

    # Create the simulator.
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    # Simulate.
    meshcat.StartRecording()
    simulator.set_monitor(_ProgressBar(scenario.simulation_duration))
    simulator.AdvanceTo(scenario.simulation_duration)
    meshcat.PublishRecording()


if __name__ == "__main__":
    main()
