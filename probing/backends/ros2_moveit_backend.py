import math


class ROS2MoveItBackend:
    """MoveIt backend for FR3 pose goals and retimed Cartesian TCP paths."""

    PILZ_PIPELINE_ID = "pilz_industrial_motion_planner"
    PILZ_PLANNER_ID = "LIN"
    PILZ_MAX_TRANS_VEL_MPS = 0.10
    PILZ_MAX_TRANS_ACC_MPS2 = 0.50
    PILZ_MAX_ROT_VEL_RADPS = 2.0
    PILZ_MAX_LINE_ERROR_M = 1e-4
    PILZ_MAX_PLANNED_SPEED_ERROR = 0.02

    def __init__(
        self,
        planning_group="fr3_arm",
        base_frame="fr3_link0",
        tip_link="spatula_tip",
        controller_names=("fr3_arm_controller",),
        node_name="probing_moveit_backend",
        move_group_action="/move_action",
        cartesian_path_service="/compute_cartesian_path",
        fk_service="/compute_fk",
        execute_trajectory_action="/execute_trajectory",
        backend_mode="auto",
        pipeline_id="move_group",
        planner_id="RRTConnectkConfigDefault",
        planning_time=5.0,
        position_tolerance=0.001,
        orientation_tolerance=0.035,
        max_velocity_scaling_factor=0.1,
        max_acceleration_scaling_factor=0.1,
        cartesian_max_step=0.001,
        cartesian_jump_threshold=0.0,
        cartesian_revolute_jump_threshold=0.2,
        cartesian_min_fraction=0.999,
        cartesian_avoid_collisions=True,
        cartesian_speed_mps=0.01,
        cartesian_acceleration_mps2=0.05,
        cartesian_angular_speed_radps=0.2,
        cartesian_angular_acceleration_radps2=0.5,
        action_timeout_s=30.0,
    ):
        self.planning_group = planning_group
        self.base_frame = base_frame
        self.tip_link = tip_link
        self.controller_names = list(controller_names or [])
        self.backend_mode = backend_mode
        self.pipeline_id = pipeline_id
        self.planner_id = planner_id
        self.planning_time = float(planning_time)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)
        self.max_velocity_scaling_factor = float(max_velocity_scaling_factor)
        self.max_acceleration_scaling_factor = float(max_acceleration_scaling_factor)
        self.cartesian_max_step = float(cartesian_max_step)
        self.cartesian_jump_threshold = float(cartesian_jump_threshold)
        self.cartesian_revolute_jump_threshold = float(
            cartesian_revolute_jump_threshold
        )
        self.cartesian_min_fraction = float(cartesian_min_fraction)
        self.cartesian_avoid_collisions = bool(cartesian_avoid_collisions)
        self.cartesian_speed_mps = float(cartesian_speed_mps)
        self.cartesian_acceleration_mps2 = float(cartesian_acceleration_mps2)
        self.cartesian_angular_speed_radps = float(cartesian_angular_speed_radps)
        self.cartesian_angular_acceleration_radps2 = float(
            cartesian_angular_acceleration_radps2
        )
        self.action_timeout_s = float(action_timeout_s)

        if self.cartesian_max_step <= 0.0:
            raise ValueError("cartesian_max_step must be positive.")
        if self.cartesian_jump_threshold < 0.0:
            raise ValueError("cartesian_jump_threshold must be non-negative.")
        if self.cartesian_revolute_jump_threshold <= 0.0:
            raise ValueError(
                "cartesian_revolute_jump_threshold must be positive."
            )
        if not 0.0 < self.cartesian_min_fraction <= 1.0:
            raise ValueError("cartesian_min_fraction must be in (0, 1].")
        if self.cartesian_speed_mps < 0.0:
            raise ValueError("cartesian_speed_mps must be non-negative.")
        if self.cartesian_acceleration_mps2 <= 0.0:
            raise ValueError("cartesian_acceleration_mps2 must be positive.")
        if self.cartesian_angular_speed_radps <= 0.0:
            raise ValueError("cartesian_angular_speed_radps must be positive.")
        if self.cartesian_angular_acceleration_radps2 <= 0.0:
            raise ValueError(
                "cartesian_angular_acceleration_radps2 must be positive."
            )

        if backend_mode not in ("auto", "moveit_py", "move_group_action"):
            raise ValueError(
                "backend_mode must be 'auto', 'moveit_py', or "
                f"'move_group_action', got {backend_mode!r}."
            )

        if backend_mode == "move_group_action":
            self._init_move_group_action(
                node_name,
                move_group_action,
                cartesian_path_service,
                fk_service,
                execute_trajectory_action,
            )
            return

        try:
            from moveit.planning import MoveItPy
        except ModuleNotFoundError as exc:
            if exc.name != "moveit" or backend_mode == "moveit_py":
                raise
            self._init_move_group_action(
                node_name,
                move_group_action,
                cartesian_path_service,
                fk_service,
                execute_trajectory_action,
            )
        else:
            self._mode = "moveit_py"
            self.moveit = MoveItPy(node_name=node_name)
            self.arm = self.moveit.get_planning_component(planning_group)

    def _init_move_group_action(
        self,
        node_name,
        move_group_action,
        cartesian_path_service,
        fk_service,
        execute_trajectory_action,
    ):
        import rclpy
        from rclpy.action import ActionClient
        from moveit_msgs.action import ExecuteTrajectory, MoveGroup
        from moveit_msgs.srv import GetCartesianPath, GetPositionFK

        self._mode = "move_group_action"
        self._move_group_action = move_group_action
        self._cartesian_path_service = cartesian_path_service
        self._fk_service = fk_service
        self._execute_trajectory_action = execute_trajectory_action
        self.node = rclpy.create_node(node_name, use_global_arguments=False)
        self._move_group_client = ActionClient(
            self.node,
            MoveGroup,
            move_group_action,
        )
        if not self._move_group_client.wait_for_server(
            timeout_sec=self.action_timeout_s
        ):
            raise RuntimeError(
                f"MoveIt action server {move_group_action!r} is not available."
            )
        self._cartesian_path_client = self.node.create_client(
            GetCartesianPath,
            cartesian_path_service,
        )
        if not self._cartesian_path_client.wait_for_service(
            timeout_sec=self.action_timeout_s
        ):
            raise RuntimeError(
                "MoveIt Cartesian path service "
                f"{cartesian_path_service!r} is not available."
            )
        self._fk_client = self.node.create_client(GetPositionFK, fk_service)
        if not self._fk_client.wait_for_service(timeout_sec=self.action_timeout_s):
            raise RuntimeError(
                f"MoveIt FK service {fk_service!r} is not available."
            )
        self._execute_trajectory_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            execute_trajectory_action,
        )
        if not self._execute_trajectory_client.wait_for_server(
            timeout_sec=self.action_timeout_s
        ):
            raise RuntimeError(
                "MoveIt trajectory action server "
                f"{execute_trajectory_action!r} is not available."
            )

    def move_to_pose(
        self,
        x_m,
        y_m,
        z_m,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        use_orientation_constraint=True,
        execute=True,
    ):
        if self._mode == "moveit_py":
            return self._move_to_pose_moveitpy(
                x_m,
                y_m,
                z_m,
                qx,
                qy,
                qz,
                qw,
                use_orientation_constraint,
                execute,
            )

        return self._move_to_pose_action(
            x_m,
            y_m,
            z_m,
            qx,
            qy,
            qz,
            qw,
            use_orientation_constraint,
            execute,
        )

    def move_to_pose_cartesian(
        self,
        x_m,
        y_m,
        z_m,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        execute=True,
    ):
        """Plan a straight TCP segment from the current state and execute it."""
        if self._mode != "move_group_action":
            raise NotImplementedError(
                "Cartesian contact stages currently require "
                "backend_mode=move_group_action."
            )

        import rclpy
        from geometry_msgs.msg import Pose
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import MoveItErrorCodes
        from moveit_msgs.srv import GetCartesianPath

        waypoint = Pose()
        waypoint.position.x = float(x_m)
        waypoint.position.y = float(y_m)
        waypoint.position.z = float(z_m)
        waypoint.orientation.x = float(qx)
        waypoint.orientation.y = float(qy)
        waypoint.orientation.z = float(qz)
        waypoint.orientation.w = float(qw)

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.start_state.is_diff = True
        request.group_name = self.planning_group
        request.link_name = self.tip_link
        request.waypoints = [waypoint]
        request.max_step = self.cartesian_max_step
        request.jump_threshold = self.cartesian_jump_threshold
        request.revolute_jump_threshold = self.cartesian_revolute_jump_threshold
        request.avoid_collisions = self.cartesian_avoid_collisions
        request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = (
            self.max_acceleration_scaling_factor
        )
        if self.cartesian_speed_mps > 0.0:
            request.cartesian_speed_limited_link = self.tip_link
            request.max_cartesian_speed = self.cartesian_speed_mps

        response_future = self._cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            response_future,
            timeout_sec=self.action_timeout_s,
        )
        if not response_future.done():
            raise TimeoutError(
                "Timed out waiting for Cartesian path service "
                f"{self._cartesian_path_service!r}."
            )

        response = response_future.result()
        if response is None:
            raise RuntimeError("MoveIt Cartesian path service returned no response.")
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            message = response.error_code.message or "no detailed MoveIt message"
            raise RuntimeError(
                "MoveIt failed to compute Cartesian path to "
                f"({x_m:.3f}, {y_m:.3f}, {z_m:.3f}): "
                f"{response.error_code.val} ({message})."
            )
        if response.fraction < self.cartesian_min_fraction:
            raise RuntimeError(
                "Refusing partial Cartesian path to "
                f"({x_m:.3f}, {y_m:.3f}, {z_m:.3f}): "
                f"fraction={response.fraction:.4f}, required>="
                f"{self.cartesian_min_fraction:.4f}."
            )

        trajectory = response.solution
        point_count = len(trajectory.joint_trajectory.points)
        if point_count < 2:
            raise RuntimeError(
                "MoveIt Cartesian path contains fewer than two trajectory points."
            )
        if self.cartesian_speed_mps > 0.0:
            self._retime_cartesian_trajectory(trajectory)
        final_time = trajectory.joint_trajectory.points[-1].time_from_start
        duration_s = float(final_time.sec) + float(final_time.nanosec) * 1e-9
        speed_text = (
            f"{self.cartesian_speed_mps:.4f} m/s"
            if self.cartesian_speed_mps > 0.0
            else "disabled"
        )
        print(
            "Cartesian path accepted: "
            f"fraction={response.fraction:.4f}, points={point_count}, "
            f"duration={duration_s:.3f} s, max_step={self.cartesian_max_step:.4f} m, "
            f"TCP speed limit={speed_text}."
        )
        if not execute:
            return response

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = trajectory
        execute_goal.controller_names = self.controller_names
        send_goal_future = self._execute_trajectory_client.send_goal_async(
            execute_goal
        )
        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future,
            timeout_sec=self.action_timeout_s,
        )
        if not send_goal_future.done():
            raise TimeoutError(
                "Timed out sending trajectory to "
                f"{self._execute_trajectory_action!r}."
            )

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the Cartesian trajectory.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=self.action_timeout_s,
        )
        if not result_future.done():
            raise TimeoutError(
                "Timed out executing Cartesian trajectory through "
                f"{self._execute_trajectory_action!r}."
            )

        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            message = result.error_code.message or "no detailed MoveIt message"
            raise RuntimeError(
                "MoveIt failed to execute Cartesian trajectory: "
                f"{result.error_code.val} ({message})."
            )
        return response

    def move_to_pose_pilz_lin(
        self,
        x_m,
        y_m,
        z_m,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        motion_kind="linear",
        execute=True,
    ):
        """Plan a Cartesian line with Pilz LIN and execute it."""
        if self._mode != "move_group_action":
            raise NotImplementedError(
                "Pilz LIN contact stages require backend_mode="
                "move_group_action."
            )

        velocity_scaling, acceleration_scaling = (
            self._pilz_scaling_factors(motion_kind)
        )
        result = self._move_to_pose_action(
            x_m,
            y_m,
            z_m,
            qx,
            qy,
            qz,
            qw,
            True,
            False,
            pipeline_id=self.PILZ_PIPELINE_ID,
            planner_id=self.PILZ_PLANNER_ID,
            velocity_scaling_factor=velocity_scaling,
            acceleration_scaling_factor=acceleration_scaling,
        )

        trajectory = result.planned_trajectory
        validation = self._validate_pilz_trajectory(trajectory, motion_kind)
        if execute:
            self._execute_robot_trajectory(
                trajectory,
                context="Pilz LIN trajectory",
            )

        points = trajectory.joint_trajectory.points
        duration_s = 0.0
        if points:
            duration = points[-1].time_from_start
            duration_s = float(duration.sec) + float(duration.nanosec) * 1e-9
        speed_text = (
            f"{self.cartesian_speed_mps:.4f} m/s"
            if motion_kind == "linear"
            else f"{self.cartesian_angular_speed_radps:.4f} rad/s"
        )
        print(
            "Pilz LIN path accepted: "
            f"mode={motion_kind}, points={len(points)}, "
            f"duration={duration_s:.3f} s, requested cruise={speed_text}, "
            f"planned cruise={validation['cruise_speed']:.4f} "
            f"{validation['speed_units']}, "
            f"line error={validation['line_error_m'] * 1000.0:.4f} mm, "
            f"velocity_scale={velocity_scaling:.4f}, "
            f"acceleration_scale={acceleration_scaling:.4f}."
        )
        return result

    def _validate_pilz_trajectory(self, trajectory, motion_kind):
        joint_trajectory = trajectory.joint_trajectory
        points = joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError("Pilz LIN produced fewer than two trajectory points.")

        tcp_poses = self._trajectory_tcp_poses(joint_trajectory)
        positions = [pose[0] for pose in tcp_poses]
        orientations = [pose[1] for pose in tcp_poses]
        times = [
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) * 1e-9
            for point in points
        ]
        linear_segments = [
            math.dist(first, second)
            for first, second in zip(positions, positions[1:])
        ]
        angular_segments = [
            self._quaternion_distance(first, second)
            for first, second in zip(orientations, orientations[1:])
        ]

        if motion_kind == "linear":
            distances = linear_segments
            target_speed = self.cartesian_speed_mps
            speed_units = "m/s"
            line_error_m = max(
                self._point_line_distance(point, positions[0], positions[-1])
                for point in positions
            )
        else:
            distances = angular_segments
            target_speed = self.cartesian_angular_speed_radps
            speed_units = "rad/s"
            line_error_m = max(
                math.dist(point, positions[0]) for point in positions
            )

        if line_error_m > self.PILZ_MAX_LINE_ERROR_M:
            raise RuntimeError(
                "Refusing Pilz LIN trajectory: planned TCP line/position "
                f"error is {line_error_m * 1000.0:.4f} mm, limit is "
                f"{self.PILZ_MAX_LINE_ERROR_M * 1000.0:.4f} mm."
            )

        cumulative = [0.0]
        for distance in distances:
            cumulative.append(cumulative[-1] + distance)
        total_distance = cumulative[-1]
        if total_distance <= 0.0:
            raise RuntimeError("Refusing near-zero Pilz LIN trajectory.")

        cruise_speeds = []
        for index, distance in enumerate(distances):
            duration_s = times[index + 1] - times[index]
            midpoint_fraction = (
                0.5 * (cumulative[index] + cumulative[index + 1])
                / total_distance
            )
            if duration_s > 0.0 and 0.3 <= midpoint_fraction <= 0.7:
                cruise_speeds.append(distance / duration_s)
        if not cruise_speeds:
            raise RuntimeError(
                "Could not identify cruise samples in Pilz LIN trajectory."
            )

        ordered_speeds = sorted(cruise_speeds)
        cruise_speed = ordered_speeds[len(ordered_speeds) // 2]
        speed_error = abs(cruise_speed / target_speed - 1.0)
        if speed_error > self.PILZ_MAX_PLANNED_SPEED_ERROR:
            raise RuntimeError(
                "Refusing Pilz LIN trajectory: planned cruise speed is "
                f"{cruise_speed:.4f} {speed_units}, requested "
                f"{target_speed:.4f} {speed_units}, error "
                f"{speed_error * 100.0:.2f}%."
            )

        return {
            "cruise_speed": cruise_speed,
            "speed_units": speed_units,
            "line_error_m": line_error_m,
        }

    @staticmethod
    def _point_line_distance(point, start, end):
        direction = tuple(target - origin for origin, target in zip(start, end))
        direction_norm_sq = sum(value * value for value in direction)
        if direction_norm_sq == 0.0:
            return math.dist(point, start)
        offset = tuple(value - origin for value, origin in zip(point, start))
        fraction = sum(
            delta * value for delta, value in zip(direction, offset)
        ) / direction_norm_sq
        projection = tuple(
            origin + fraction * delta for origin, delta in zip(start, direction)
        )
        return math.dist(point, projection)

    def _execute_robot_trajectory(self, trajectory, context):
        import rclpy
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import MoveItErrorCodes

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = trajectory
        execute_goal.controller_names = self.controller_names
        send_goal_future = self._execute_trajectory_client.send_goal_async(
            execute_goal
        )
        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future,
            timeout_sec=self.action_timeout_s,
        )
        if not send_goal_future.done():
            raise TimeoutError(
                f"Timed out sending {context} to "
                f"{self._execute_trajectory_action!r}."
            )
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"MoveIt rejected the {context}.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=self.action_timeout_s,
        )
        if not result_future.done():
            raise TimeoutError(
                f"Timed out executing {context} through "
                f"{self._execute_trajectory_action!r}."
            )
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            message = result.error_code.message or "no detailed MoveIt message"
            raise RuntimeError(
                f"MoveIt failed to execute {context}: "
                f"{result.error_code.val} ({message})."
            )
        return result

    def _pilz_scaling_factors(self, motion_kind):
        if motion_kind == "linear":
            velocity_scaling = (
                self.cartesian_speed_mps / self.PILZ_MAX_TRANS_VEL_MPS
            )
            acceleration_scaling = (
                self.cartesian_acceleration_mps2
                / self.PILZ_MAX_TRANS_ACC_MPS2
            )
        elif motion_kind == "angular":
            velocity_scaling = (
                self.cartesian_angular_speed_radps
                / self.PILZ_MAX_ROT_VEL_RADPS
            )
            max_rot_acceleration = (
                self.PILZ_MAX_TRANS_ACC_MPS2
                / self.PILZ_MAX_TRANS_VEL_MPS
                * self.PILZ_MAX_ROT_VEL_RADPS
            )
            acceleration_scaling = (
                self.cartesian_angular_acceleration_radps2
                / max_rot_acceleration
            )
        else:
            raise ValueError(
                "Pilz motion_kind must be 'linear' or 'angular', got "
                f"{motion_kind!r}."
            )

        for label, value in (
            ("velocity", velocity_scaling),
            ("acceleration", acceleration_scaling),
        ):
            if not 0.0 < value <= 1.0:
                raise ValueError(
                    f"Requested Pilz {label} scaling is {value:.4f}; it must "
                    "be in (0, 1]. Check the requested Cartesian speed or "
                    "acceleration against the FR3 Pilz limits."
                )
        return velocity_scaling, acceleration_scaling

    def _retime_cartesian_trajectory(self, trajectory):
        tcp_poses = self._trajectory_tcp_poses(trajectory.joint_trajectory)
        linear_distances = [0.0]
        angular_distances = [0.0]
        for previous, current in zip(tcp_poses, tcp_poses[1:]):
            linear_distances.append(
                linear_distances[-1]
                + math.dist(previous[0], current[0])
            )
            angular_distances.append(
                angular_distances[-1]
                + self._quaternion_distance(previous[1], current[1])
            )

        total_linear_m = linear_distances[-1]
        total_angular_rad = angular_distances[-1]
        if total_linear_m >= 1e-5:
            distances = linear_distances
            total_distance = total_linear_m
            cruise_speed = self.cartesian_speed_mps
            acceleration = self.cartesian_acceleration_mps2
            motion_label = "linear TCP"
            units = "m"
            speed_units = "m/s"
        elif total_angular_rad >= 1e-4:
            distances = angular_distances
            total_distance = total_angular_rad
            cruise_speed = self.cartesian_angular_speed_radps
            acceleration = self.cartesian_angular_acceleration_radps2
            motion_label = "angular TCP"
            units = "rad"
            speed_units = "rad/s"
        else:
            raise RuntimeError(
                "Cannot retime Cartesian trajectory with near-zero TCP motion."
            )

        times = [
            self._trapezoidal_time_at_distance(
                distance,
                total_distance,
                cruise_speed,
                acceleration,
            )
            for distance in distances
        ]
        previous_time = -1e-6
        for point, time_s in zip(trajectory.joint_trajectory.points, times):
            time_s = max(time_s, previous_time + 1e-6)
            self._set_duration(point.time_from_start, time_s)
            point.velocities = []
            point.accelerations = []
            point.effort = []
            previous_time = time_s

        print(
            "Cartesian trapezoidal retiming: "
            f"mode={motion_label}, path={total_distance:.5f} {units}, "
            f"cruise={cruise_speed:.4f} {speed_units}, "
            f"duration={times[-1]:.3f} s."
        )

    def _trajectory_tcp_poses(self, joint_trajectory):
        import rclpy
        from moveit_msgs.msg import MoveItErrorCodes
        from moveit_msgs.srv import GetPositionFK

        poses = []
        for point in joint_trajectory.points:
            request = GetPositionFK.Request()
            request.header.frame_id = self.base_frame
            request.fk_link_names = [self.tip_link]
            request.robot_state.joint_state.name = list(
                joint_trajectory.joint_names
            )
            request.robot_state.joint_state.position = list(point.positions)
            response_future = self._fk_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node,
                response_future,
                timeout_sec=self.action_timeout_s,
            )
            if not response_future.done():
                raise TimeoutError(
                    f"Timed out waiting for FK service {self._fk_service!r}."
                )
            response = response_future.result()
            if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
                code = response.error_code.val if response is not None else "none"
                raise RuntimeError(
                    "MoveIt FK failed while retiming Cartesian trajectory: "
                    f"error={code}."
                )
            if not response.pose_stamped:
                raise RuntimeError("MoveIt FK returned no TCP pose.")
            pose = response.pose_stamped[0].pose
            poses.append(
                (
                    (pose.position.x, pose.position.y, pose.position.z),
                    (
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
            )
        return poses

    @staticmethod
    def _quaternion_distance(first, second):
        first_norm = math.sqrt(sum(value * value for value in first))
        second_norm = math.sqrt(sum(value * value for value in second))
        if first_norm == 0.0 or second_norm == 0.0:
            raise RuntimeError("FK returned a zero-norm TCP quaternion.")
        dot = abs(sum(a * b for a, b in zip(first, second))) / (
            first_norm * second_norm
        )
        return 2.0 * math.acos(max(-1.0, min(1.0, dot)))

    @staticmethod
    def _trapezoidal_time_at_distance(
        distance,
        total_distance,
        cruise_speed,
        acceleration,
    ):
        distance = min(max(distance, 0.0), total_distance)
        peak_speed = min(cruise_speed, math.sqrt(total_distance * acceleration))
        acceleration_distance = peak_speed * peak_speed / (2.0 * acceleration)
        acceleration_time = peak_speed / acceleration
        cruise_distance = max(total_distance - 2.0 * acceleration_distance, 0.0)
        total_time = 2.0 * acceleration_time + cruise_distance / peak_speed

        if distance <= acceleration_distance:
            return math.sqrt(2.0 * distance / acceleration)
        if distance < total_distance - acceleration_distance:
            return acceleration_time + (
                distance - acceleration_distance
            ) / peak_speed
        remaining_distance = max(total_distance - distance, 0.0)
        return total_time - math.sqrt(2.0 * remaining_distance / acceleration)

    @staticmethod
    def _set_duration(duration, time_s):
        seconds = int(math.floor(time_s))
        nanoseconds = int(round((time_s - seconds) * 1e9))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        duration.sec = seconds
        duration.nanosec = nanoseconds

    def _move_to_pose_moveitpy(
        self,
        x_m,
        y_m,
        z_m,
        qx,
        qy,
        qz,
        qw,
        use_orientation_constraint,
        execute,
    ):
        from geometry_msgs.msg import PoseStamped

        if not use_orientation_constraint:
            raise NotImplementedError(
                "Position-only goals are implemented for backend_mode="
                "move_group_action. Use --backend-mode move_group_action."
            )

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.base_frame

        pose_goal.pose.orientation.x = float(qx)
        pose_goal.pose.orientation.y = float(qy)
        pose_goal.pose.orientation.z = float(qz)
        pose_goal.pose.orientation.w = float(qw)

        pose_goal.pose.position.x = float(x_m)
        pose_goal.pose.position.y = float(y_m)
        pose_goal.pose.position.z = float(z_m)

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=self.tip_link,
        )

        plan_result = self.arm.plan()
        if not plan_result:
            raise RuntimeError(
                "MoveIt failed to plan to pose "
                f"({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) in {self.base_frame}."
            )

        if execute:
            self.moveit.execute(
                plan_result.trajectory,
                controllers=self.controller_names,
            )

        return plan_result

    def _move_to_pose_action(
        self,
        x_m,
        y_m,
        z_m,
        qx,
        qy,
        qz,
        qw,
        use_orientation_constraint,
        execute,
        pipeline_id=None,
        planner_id=None,
        velocity_scaling_factor=None,
        acceleration_scaling_factor=None,
    ):
        import rclpy
        from geometry_msgs.msg import Pose
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            Constraints,
            MoveItErrorCodes,
            OrientationConstraint,
            PositionConstraint,
        )
        from shape_msgs.msg import SolidPrimitive

        pose_goal = Pose()
        pose_goal.orientation.x = float(qx)
        pose_goal.orientation.y = float(qy)
        pose_goal.orientation.z = float(qz)
        pose_goal.orientation.w = float(qw)
        pose_goal.position.x = float(x_m)
        pose_goal.position.y = float(y_m)
        pose_goal.position.z = float(z_m)

        position_region = Pose()
        position_region.orientation.w = 1.0
        position_region.position = pose_goal.position

        position_bounds = SolidPrimitive()
        position_bounds.type = SolidPrimitive.SPHERE
        position_bounds.dimensions = [self.position_tolerance]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.tip_link
        position_constraint.constraint_region.primitives = [position_bounds]
        position_constraint.constraint_region.primitive_poses = [position_region]
        position_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.name = "fr3_probe_position_goal"
        goal_constraints.position_constraints = [position_constraint]

        if use_orientation_constraint:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame
            orientation_constraint.link_name = self.tip_link
            orientation_constraint.orientation = pose_goal.orientation
            orientation_constraint.absolute_x_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_y_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_z_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.parameterization = (
                OrientationConstraint.ROTATION_VECTOR
            )
            orientation_constraint.weight = 1.0
            goal_constraints.name = "fr3_probe_pose_goal"
            goal_constraints.orientation_constraints = [orientation_constraint]

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.min_corner.z = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.max_corner.z = 2.0
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints = [goal_constraints]
        selected_pipeline = pipeline_id or self.pipeline_id
        selected_planner = planner_id or self.planner_id
        selected_velocity_scaling = (
            self.max_velocity_scaling_factor
            if velocity_scaling_factor is None
            else float(velocity_scaling_factor)
        )
        selected_acceleration_scaling = (
            self.max_acceleration_scaling_factor
            if acceleration_scaling_factor is None
            else float(acceleration_scaling_factor)
        )
        goal_msg.request.pipeline_id = selected_pipeline
        goal_msg.request.planner_id = selected_planner
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = selected_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = (
            selected_acceleration_scaling
        )
        goal_msg.planning_options.plan_only = not execute
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future,
            timeout_sec=self.action_timeout_s,
        )
        if not send_goal_future.done():
            raise TimeoutError(
                f"Timed out sending goal to {self._move_group_action!r}."
            )

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the pose goal.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=self.action_timeout_s,
        )
        if not result_future.done():
            raise TimeoutError(
                f"Timed out waiting for {self._move_group_action!r} result."
            )

        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            message = result.error_code.message or "no detailed MoveIt message"
            raise RuntimeError(
                "MoveIt failed to plan/execute pose "
                f"({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) in {self.base_frame}: "
                f"{result.error_code.val} ({message}); "
                f"pipeline={selected_pipeline}, planner={selected_planner}."
            )

        return result
