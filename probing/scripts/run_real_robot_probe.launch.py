import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_parameter_name = "robot_ip"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    allow_contact_primitives_parameter_name = "allow_contact_primitives"
    namespace_parameter_name = "namespace"
    probe_primitive_parameter_name = "probe_primitive"
    probe_sequence_parameter_name = "probe_sequence"
    probe_x_parameter_name = "probe_x"
    probe_y_parameter_name = "probe_y"
    approach_z_parameter_name = "approach_z"
    smoke_down_z_parameter_name = "smoke_down_z"
    compression_depth_parameter_name = "compression_depth"
    insert_depth_parameter_name = "insert_depth"
    drag_distance_parameter_name = "drag_distance"
    lift_height_parameter_name = "lift_height"
    tilt_deg_parameter_name = "tilt_deg"
    hold_s_parameter_name = "hold_s"
    planning_group_parameter_name = "planning_group"
    base_frame_parameter_name = "base_frame"
    tip_link_parameter_name = "tip_link"
    controller_parameter_name = "controller"
    backend_mode_parameter_name = "backend_mode"
    contact_stage_mode_parameter_name = "contact_stage_mode"
    cartesian_max_step_parameter_name = "cartesian_max_step"
    cartesian_min_fraction_parameter_name = "cartesian_min_fraction"
    cartesian_revolute_jump_threshold_parameter_name = (
        "cartesian_revolute_jump_threshold"
    )
    cartesian_speed_mps_parameter_name = "cartesian_speed_mps"
    cartesian_acceleration_mps2_parameter_name = "cartesian_acceleration_mps2"
    cartesian_angular_speed_radps_parameter_name = (
        "cartesian_angular_speed_radps"
    )
    cartesian_angular_acceleration_radps2_parameter_name = (
        "cartesian_angular_acceleration_radps2"
    )
    telemetry_log_path_parameter_name = "telemetry_log_path"
    wrench_topic_parameter_name = "wrench_topic"
    robot_state_topic_parameter_name = "robot_state_topic"
    telemetry_sample_period_s_parameter_name = "telemetry_sample_period_s"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    allow_contact_primitives = LaunchConfiguration(
        allow_contact_primitives_parameter_name
    )
    namespace = LaunchConfiguration(namespace_parameter_name)
    probe_primitive = LaunchConfiguration(probe_primitive_parameter_name)
    probe_sequence = LaunchConfiguration(probe_sequence_parameter_name)
    probe_x = LaunchConfiguration(probe_x_parameter_name)
    probe_y = LaunchConfiguration(probe_y_parameter_name)
    approach_z = LaunchConfiguration(approach_z_parameter_name)
    smoke_down_z = LaunchConfiguration(smoke_down_z_parameter_name)
    compression_depth = LaunchConfiguration(compression_depth_parameter_name)
    insert_depth = LaunchConfiguration(insert_depth_parameter_name)
    drag_distance = LaunchConfiguration(drag_distance_parameter_name)
    lift_height = LaunchConfiguration(lift_height_parameter_name)
    tilt_deg = LaunchConfiguration(tilt_deg_parameter_name)
    hold_s = LaunchConfiguration(hold_s_parameter_name)
    planning_group = LaunchConfiguration(planning_group_parameter_name)
    base_frame = LaunchConfiguration(base_frame_parameter_name)
    tip_link = LaunchConfiguration(tip_link_parameter_name)
    controller = LaunchConfiguration(controller_parameter_name)
    backend_mode = LaunchConfiguration(backend_mode_parameter_name)
    contact_stage_mode = LaunchConfiguration(contact_stage_mode_parameter_name)
    cartesian_max_step = LaunchConfiguration(cartesian_max_step_parameter_name)
    cartesian_min_fraction = LaunchConfiguration(
        cartesian_min_fraction_parameter_name
    )
    cartesian_revolute_jump_threshold = LaunchConfiguration(
        cartesian_revolute_jump_threshold_parameter_name
    )
    cartesian_speed_mps = LaunchConfiguration(cartesian_speed_mps_parameter_name)
    cartesian_acceleration_mps2 = LaunchConfiguration(
        cartesian_acceleration_mps2_parameter_name
    )
    cartesian_angular_speed_radps = LaunchConfiguration(
        cartesian_angular_speed_radps_parameter_name
    )
    cartesian_angular_acceleration_radps2 = LaunchConfiguration(
        cartesian_angular_acceleration_radps2_parameter_name
    )
    telemetry_log_path = LaunchConfiguration(telemetry_log_path_parameter_name)
    wrench_topic = LaunchConfiguration(wrench_topic_parameter_name)
    robot_state_topic = LaunchConfiguration(robot_state_topic_parameter_name)
    telemetry_sample_period_s = LaunchConfiguration(
        telemetry_sample_period_s_parameter_name
    )

    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parents[1]
    script_path = script_dir / "run_real_robot_probe.py"
    moveit_py_params = script_dir / "moveit_py.yaml"

    python_path = str(project_root)
    if os.environ.get("PYTHONPATH"):
        python_path = python_path + os.pathsep + os.environ["PYTHONPATH"]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                default_value="",
                description=(
                    "FR3 robot IP metadata. Start the official "
                    "franka_fr3_moveit_config real-hardware launch with the "
                    "same IP before running this connect-only probe launch."
                ),
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description=(
                    "Hardware-mode metadata for the probing node. This launch "
                    "defaults to real-hardware preflight."
                ),
            ),
            DeclareLaunchArgument(
                allow_contact_primitives_parameter_name,
                default_value="false",
                description=(
                    "Safety gate. Keep false for real-robot preflight; only "
                    "approach_smoke is allowed unless explicitly set true "
                    "after TCP, F/T logging, and abort checks are ready."
                ),
            ),
            DeclareLaunchArgument(
                namespace_parameter_name,
                default_value="",
                description="Namespace for the probing node.",
            ),
            DeclareLaunchArgument(
                probe_primitive_parameter_name,
                default_value="approach_smoke",
                description=(
                    "FR3 entry: approach_smoke, trajectory_smoke, "
                    "micro_compression, horizontal_drag, lift_detach, "
                    "or inclined_insertion."
                ),
            ),
            DeclareLaunchArgument(
                probe_sequence_parameter_name,
                default_value="approach",
                description=(
                    "Backward-compatible smoke sequence: approach "
                    "or approach_down_retract."
                ),
            ),
            DeclareLaunchArgument(
                probe_x_parameter_name,
                default_value="0.40",
                description="FR3 probing target x in fr3_link0.",
            ),
            DeclareLaunchArgument(
                probe_y_parameter_name,
                default_value="0.00",
                description="FR3 probing target y in fr3_link0.",
            ),
            DeclareLaunchArgument(
                approach_z_parameter_name,
                default_value="0.55",
                description=(
                    "High, non-contact FR3 preflight approach z in fr3_link0."
                ),
            ),
            DeclareLaunchArgument(
                smoke_down_z_parameter_name,
                default_value="0.43",
                description="FR3 smoke DOWN z in fr3_link0.",
            ),
            DeclareLaunchArgument(
                compression_depth_parameter_name,
                default_value="0.005",
                description="FR3 compression pose decrement.",
            ),
            DeclareLaunchArgument(
                insert_depth_parameter_name,
                default_value="0.005",
                description="FR3 insertion pose decrement.",
            ),
            DeclareLaunchArgument(
                drag_distance_parameter_name,
                default_value="-0.030",
                description="FR3 horizontal_drag distance along fr3_link0 x.",
            ),
            DeclareLaunchArgument(
                lift_height_parameter_name,
                default_value="0.040",
                description="FR3 lift_detach upward lift from compressed pose.",
            ),
            DeclareLaunchArgument(
                tilt_deg_parameter_name,
                default_value="12.0",
                description=(
                    "FR3 inclined_insertion tilt angle in degrees; allowed "
                    "range is 10-15."
                ),
            ),
            DeclareLaunchArgument(
                hold_s_parameter_name,
                default_value="1.0",
                description="Hold duration after contact-style stages.",
            ),
            DeclareLaunchArgument(
                planning_group_parameter_name,
                default_value="fr3_arm",
                description="MoveIt planning group for FR3 probing.",
            ),
            DeclareLaunchArgument(
                base_frame_parameter_name,
                default_value="fr3_link0",
                description="Base frame for FR3 probing pose goals.",
            ),
            DeclareLaunchArgument(
                tip_link_parameter_name,
                default_value="spatula_tip",
                description="MoveIt pose goal link for FR3 probing.",
            ),
            DeclareLaunchArgument(
                controller_parameter_name,
                default_value="fr3_arm_controller",
                description="MoveIt execution controller for FR3 probing.",
            ),
            DeclareLaunchArgument(
                backend_mode_parameter_name,
                default_value="move_group_action",
                description=(
                    "Backend mode: move_group_action, auto, or moveit_py. "
                    "Use move_group_action to connect to an existing FR3 "
                    "MoveIt pipeline."
                ),
            ),
            DeclareLaunchArgument(
                contact_stage_mode_parameter_name,
                default_value="pilz_lin",
                description=(
                    "Use Pilz LIN straight paths with trapezoidal timing for "
                    "probing stages."
                ),
            ),
            DeclareLaunchArgument(
                cartesian_max_step_parameter_name,
                default_value="0.001",
                description="Maximum Cartesian interpolation step in metres.",
            ),
            DeclareLaunchArgument(
                cartesian_min_fraction_parameter_name,
                default_value="0.999",
                description="Minimum complete Cartesian fraction to execute.",
            ),
            DeclareLaunchArgument(
                cartesian_revolute_jump_threshold_parameter_name,
                default_value="0.2",
                description="Absolute revolute joint jump limit in radians.",
            ),
            DeclareLaunchArgument(
                cartesian_speed_mps_parameter_name,
                default_value="0.010",
                description="Cartesian probing-stage TCP speed limit in m/s.",
            ),
            DeclareLaunchArgument(
                cartesian_acceleration_mps2_parameter_name,
                default_value="0.050",
                description="Cartesian probing-stage acceleration in m/s^2.",
            ),
            DeclareLaunchArgument(
                cartesian_angular_speed_radps_parameter_name,
                default_value="0.20",
                description="Pure rotation cruise speed in rad/s.",
            ),
            DeclareLaunchArgument(
                cartesian_angular_acceleration_radps2_parameter_name,
                default_value="0.50",
                description="Pure rotation acceleration in rad/s^2.",
            ),
            DeclareLaunchArgument(
                telemetry_log_path_parameter_name,
                default_value="",
                description=(
                    "Optional CSV output path for stage telemetry. Leave empty "
                    "to disable logging."
                ),
            ),
            DeclareLaunchArgument(
                wrench_topic_parameter_name,
                default_value=(
                    "/franka_robot_state_broadcaster/"
                    "external_wrench_in_base_frame"
                ),
                description="Optional WrenchStamped topic for F/T logging.",
            ),
            DeclareLaunchArgument(
                robot_state_topic_parameter_name,
                default_value="/franka_robot_state_broadcaster/robot_state",
                description=(
                    "Optional FrankaRobotState topic used as fallback for "
                    "wrench and measured EE pose."
                ),
            ),
            DeclareLaunchArgument(
                telemetry_sample_period_s_parameter_name,
                default_value="0.02",
                description="Requested TCP/F/T sampling period during every stage.",
            ),
            Node(
                executable="python3",
                name="run_real_robot_probe",
                namespace=namespace,
                output="screen",
                arguments=[
                    str(script_path),
                    "--primitive",
                    probe_primitive,
                    "--sequence",
                    probe_sequence,
                    "--x",
                    probe_x,
                    "--y",
                    probe_y,
                    "--approach-z",
                    approach_z,
                    "--smoke-down-z",
                    smoke_down_z,
                    "--compression-depth",
                    compression_depth,
                    "--insert-depth",
                    insert_depth,
                    "--drag-distance",
                    drag_distance,
                    "--lift-height",
                    lift_height,
                    "--tilt-deg",
                    tilt_deg,
                    "--hold-s",
                    hold_s,
                    "--planning-group",
                    planning_group,
                    "--base-frame",
                    base_frame,
                    "--tip-link",
                    tip_link,
                    "--controller",
                    controller,
                    "--backend-mode",
                    backend_mode,
                    "--contact-stage-mode",
                    contact_stage_mode,
                    "--cartesian-max-step",
                    cartesian_max_step,
                    "--cartesian-min-fraction",
                    cartesian_min_fraction,
                    "--cartesian-revolute-jump-threshold",
                    cartesian_revolute_jump_threshold,
                    "--cartesian-speed-mps",
                    cartesian_speed_mps,
                    "--cartesian-acceleration-mps2",
                    cartesian_acceleration_mps2,
                    "--cartesian-angular-speed-radps",
                    cartesian_angular_speed_radps,
                    "--cartesian-angular-acceleration-radps2",
                    cartesian_angular_acceleration_radps2,
                    "--robot-ip",
                    robot_ip,
                    "--use-fake-hardware",
                    use_fake_hardware,
                    "--allow-contact-primitives",
                    allow_contact_primitives,
                    "--telemetry-log-path",
                    telemetry_log_path,
                    "--wrench-topic",
                    wrench_topic,
                    "--robot-state-topic",
                    robot_state_topic,
                    "--telemetry-sample-period-s",
                    telemetry_sample_period_s,
                ],
                parameters=[str(moveit_py_params)],
                additional_env={
                    "PYTHONPATH": python_path,
                    "PYTHONUNBUFFERED": "1",
                },
            ),
        ]
    )
