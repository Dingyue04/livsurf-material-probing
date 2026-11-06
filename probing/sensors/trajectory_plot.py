import math
from pathlib import Path


class ProbeTrajectoryPlotter:
    """Continuously record TCP TF samples and save a 3D trajectory plot."""

    def __init__(
        self,
        node,
        output_path,
        primitive,
        base_frame,
        tip_link,
        sample_period_s=0.02,
    ):
        self.node = node
        self.output_path = Path(output_path)
        self.primitive = primitive
        self.base_frame = base_frame
        self.tip_link = tip_link
        self.sample_period_s = float(sample_period_s)
        self.records = []
        self._active_stage = None
        self._active_command_pose = None
        self._init_tf()
        self._sample_timer = self.node.create_timer(
            self.sample_period_s,
            self._on_sample_timer,
        )

        print(
            "Trajectory plot enabled: "
            f"path={self.output_path}, tip_link={self.tip_link}."
        )

    @classmethod
    def from_args(cls, node, args, primitive):
        if not args.trajectory_plot_path:
            return None

        return cls(
            node=node,
            output_path=args.trajectory_plot_path,
            primitive=primitive,
            base_frame=args.base_frame,
            tip_link=args.tip_link,
            sample_period_s=args.telemetry_sample_period_s,
        )

    def _init_tf(self):
        try:
            from tf2_ros import Buffer, TransformListener
        except ModuleNotFoundError:
            self.tf_buffer = None
            self.tf_listener = None
            return

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=False,
        )

    def warmup(self, duration_s=0.25):
        import rclpy
        import time

        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def capture(self, stage, event="sample", command_pose=None):
        if event == "start":
            self._active_stage = stage
            self._active_command_pose = command_pose

        self._record(stage, event=event, command_pose=command_pose)

        if event == "done" and stage == self._active_stage:
            self._active_stage = None
            self._active_command_pose = None

    def _on_sample_timer(self):
        if self._active_stage is None:
            return
        self._record(
            self._active_stage,
            event="sample",
            command_pose=self._active_command_pose,
        )

    def _record(self, stage, event, command_pose):
        pose = self._lookup_tf_pose()
        source = "tf"
        if pose is None and command_pose is not None and event != "sample":
            pose = (None, *command_pose[:3])
            source = "commanded_pose_fallback"

        if pose is None:
            print(
                "WARNING: Could not record trajectory point for "
                f"{stage}; no TF or commanded pose is available."
            )
            return

        command_xyz = command_pose[:3] if command_pose is not None else None
        stamp_s, x_m, y_m, z_m = pose
        if (
            event == "sample"
            and stamp_s is not None
            and self.records
            and self.records[-1]["stage"] == stage
            and self.records[-1]["stamp_s"] == stamp_s
        ):
            return

        self.records.append(
            {
                "stage": stage,
                "event": event,
                "stamp_s": stamp_s,
                "x": float(x_m),
                "y": float(y_m),
                "z": float(z_m),
                "source": source,
                "command_xyz": command_xyz,
            }
        )

    def _lookup_tf_pose(self):
        if self.tf_buffer is None:
            return None

        try:
            from rclpy.time import Time

            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tip_link,
                Time(),
            )
        except Exception:
            return None

        translation = transform.transform.translation
        stamp = transform.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return (stamp_s, translation.x, translation.y, translation.z)

    def close(self):
        if self._sample_timer is not None:
            self.node.destroy_timer(self._sample_timer)
            self._sample_timer = None
        if not self.output_path:
            return
        if not self.records:
            print("WARNING: No trajectory stage points were recorded.")
            return

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        suffix = self.output_path.suffix.lower()
        if suffix == ".png":
            try:
                self._save_png()
            except ModuleNotFoundError as exc:
                fallback_path = self.output_path.with_suffix(".svg")
                print(
                    "WARNING: matplotlib is not available "
                    f"({exc.name}); saving SVG fallback to {fallback_path}."
                )
                self.output_path = fallback_path
                self._save_svg()
        else:
            self._save_svg()

        print(
            "Trajectory plot saved: "
            f"{self.output_path} ({len(self.records)} unique TCP/event samples)."
        )

    def _save_png(self):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        xs = [record["x"] for record in self.records]
        ys = [record["y"] for record in self.records]
        zs = [record["z"] for record in self.records]

        fig = plt.figure(figsize=(12.5, 7.0))
        ax = fig.add_axes([0.04, 0.10, 0.66, 0.80], projection="3d")
        ax.plot(xs, ys, zs, linewidth=2.0, label="actual TCP samples")

        stage_records = [
            record for record in self.records if record["event"] == "done"
        ]
        if not stage_records:
            stage_records = [self.records[-1]]
        ax.scatter(
            [record["x"] for record in stage_records],
            [record["y"] for record in stage_records],
            [record["z"] for record in stage_records],
            marker="o",
            s=32,
            label="stage end",
        )

        command_points = [
            record["command_xyz"]
            for record in stage_records
            if record["command_xyz"] is not None
        ]
        if command_points:
            cmd_xs = [point[0] for point in command_points]
            cmd_ys = [point[1] for point in command_points]
            cmd_zs = [point[2] for point in command_points]
            ax.plot(
                cmd_xs,
                cmd_ys,
                cmd_zs,
                linestyle="--",
                marker="x",
                linewidth=1.2,
                label="commanded pose",
            )

        for idx, record in enumerate(stage_records, start=1):
            ax.text(
                record["x"],
                record["y"],
                record["z"],
                f" {idx}",
                fontsize=10,
                fontweight="bold",
            )

        ax.set_title(f"{self.primitive}: continuous {self.tip_link} trajectory")
        ax.set_xlabel(f"x in {self.base_frame} (m)")
        ax.set_ylabel(f"y in {self.base_frame} (m)")
        ax.set_zlabel(f"z in {self.base_frame} (m)")
        ax.legend()
        self._set_equal_xyz_scale(ax, xs, ys, zs, command_points)
        ax.view_init(elev=24, azim=-58)

        table_ax = fig.add_axes([0.72, 0.16, 0.27, 0.68])
        table_ax.axis("off")
        table_ax.set_title("Actual TCP coordinates (m)", pad=12, fontweight="bold")
        table_rows = [
            [
                str(idx),
                record["stage"],
                f'{record["x"]:.4f}',
                f'{record["y"]:.4f}',
                f'{record["z"]:.4f}',
            ]
            for idx, record in enumerate(stage_records, start=1)
        ]
        table = table_ax.table(
            cellText=table_rows,
            colLabels=["#", "stage", "x", "y", "z"],
            cellLoc="center",
            loc="upper center",
            colWidths=[0.09, 0.31, 0.20, 0.20, 0.20],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.0, 1.55)
        table_ax.text(
            0.0,
            0.02,
            f"Frame: {self.base_frame}\nContinuous TF line; table shows stage ends",
            transform=table_ax.transAxes,
            fontsize=9,
            color="#4b5563",
        )
        fig.savefig(self.output_path, dpi=160)
        plt.close(fig)

    @staticmethod
    def _set_equal_xyz_scale(ax, xs, ys, zs, command_points):
        """Use one metres-per-screen-unit scale on all three plot axes."""
        all_xs = list(xs) + [point[0] for point in command_points]
        all_ys = list(ys) + [point[1] for point in command_points]
        all_zs = list(zs) + [point[2] for point in command_points]
        centers = [
            0.5 * (min(values) + max(values))
            for values in (all_xs, all_ys, all_zs)
        ]
        max_span = max(
            max(values) - min(values)
            for values in (all_xs, all_ys, all_zs)
        )
        half_span = max(0.003, 0.58 * max_span)
        ax.set_xlim(centers[0] - half_span, centers[0] + half_span)
        ax.set_ylim(centers[1] - half_span, centers[1] + half_span)
        ax.set_zlim(centers[2] - half_span, centers[2] + half_span)
        ax.set_box_aspect((1, 1, 1))

    def _save_svg(self):
        projected = [self._project(record["x"], record["y"], record["z"]) for record in self.records]
        stage_records = [
            record for record in self.records if record["event"] == "done"
        ]
        if not stage_records:
            stage_records = [self.records[-1]]
        stage_projected = [
            self._project(record["x"], record["y"], record["z"])
            for record in stage_records
        ]
        cmd_projected = [
            self._project(*record["command_xyz"])
            if record["command_xyz"] is not None
            else None
            for record in stage_records
        ]

        all_points = (
            list(projected)
            + list(stage_projected)
            + [point for point in cmd_projected if point]
        )
        raw_min_u = min(point[0] for point in all_points)
        raw_max_u = max(point[0] for point in all_points)
        raw_min_v = min(point[1] for point in all_points)
        raw_max_v = max(point[1] for point in all_points)

        width = 1280
        height = 720
        plot_width = 850
        pad = 80
        span_u = max(raw_max_u - raw_min_u, 0.08)
        span_v = max(raw_max_v - raw_min_v, 0.08)
        center_u = 0.5 * (raw_min_u + raw_max_u)
        center_v = 0.5 * (raw_min_v + raw_max_v)
        min_u = center_u - 0.5 * span_u
        min_v = center_v - 0.5 * span_v
        scale = min((plot_width - 2 * pad) / span_u, (height - 2 * pad) / span_v)

        def to_canvas(point):
            u, v = point
            x = pad + (u - min_u) * scale
            y = height - pad - (v - min_v) * scale
            return (x, y)

        canvas_points = [to_canvas(point) for point in projected]
        canvas_stage_points = [to_canvas(point) for point in stage_projected]
        canvas_cmd_points = [
            to_canvas(point) if point is not None else None for point in cmd_projected
        ]

        actual_polyline = self._polyline(canvas_points)
        command_polyline = self._polyline(
            [point for point in canvas_cmd_points if point is not None]
        )
        axes = self._svg_axes(to_canvas)

        labels = []
        for idx, (record, point) in enumerate(
            zip(stage_records, canvas_stage_points),
            start=1,
        ):
            x, y = point
            labels.append(
                f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#0f766e" />'
            )
            labels.append(
                '<text '
                f'x="{x + 8:.1f}" y="{y - 8:.1f}" '
                'font-family="Arial, sans-serif" font-size="13" '
                'fill="#111827">'
                f'{idx}:{self._escape(record["stage"])}</text>'
            )

        table_rows = []
        for idx, record in enumerate(stage_records, start=1):
            row_y = 144 + (idx - 1) * 62
            table_rows.append(
                '<text '
                f'x="890" y="{row_y}" '
                'font-family="Arial, sans-serif" font-size="15" '
                'font-weight="700" fill="#111827">'
                f'{idx}. {self._escape(record["stage"])}</text>'
            )
            table_rows.append(
                '<text '
                f'x="890" y="{row_y + 24}" '
                'font-family="monospace" font-size="14" fill="#374151">'
                f'x={record["x"]:.4f}  y={record["y"]:.4f}  '
                f'z={record["z"]:.4f} m</text>'
            )

        command_marks = []
        for point in canvas_cmd_points:
            if point is None:
                continue
            x, y = point
            command_marks.append(
                '<path '
                f'd="M {x - 5:.1f} {y - 5:.1f} L {x + 5:.1f} {y + 5:.1f} '
                f'M {x + 5:.1f} {y - 5:.1f} L {x - 5:.1f} {y + 5:.1f}" '
                'stroke="#dc2626" stroke-width="2" />'
            )

        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#ffffff" />
  <text x="40" y="42" font-family="Arial, sans-serif" font-size="24" font-weight="700" fill="#111827">{self._escape(self.primitive)} continuous {self._escape(self.tip_link)} trajectory</text>
  <text x="40" y="68" font-family="Arial, sans-serif" font-size="14" fill="#4b5563">Projected 3D view in {self._escape(self.base_frame)}. Solid teal = continuous TCP TF samples. Dashed red = stage goals.</text>
  <text x="890" y="98" font-family="Arial, sans-serif" font-size="18" font-weight="700" fill="#111827">Actual TCP stage coordinates</text>
  <line x1="870" y1="80" x2="870" y2="680" stroke="#d1d5db" stroke-width="1" />
  {axes}
  <polyline points="{command_polyline}" fill="none" stroke="#dc2626" stroke-width="2" stroke-dasharray="8 6" />
  <polyline points="{actual_polyline}" fill="none" stroke="#0f766e" stroke-width="3" />
  {''.join(command_marks)}
  {''.join(labels)}
  {''.join(table_rows)}
  <text x="890" y="660" font-family="Arial, sans-serif" font-size="13" fill="#6b7280">Table: stage ends. Line: all TF samples. Units: metres.</text>
</svg>
'''
        self.output_path.write_text(svg)

    @staticmethod
    def _project(x, y, z):
        azimuth = math.radians(-45.0)
        elevation = math.radians(24.0)
        u = math.cos(azimuth) * x - math.sin(azimuth) * y
        v = (
            math.sin(elevation) * math.sin(azimuth) * x
            + math.sin(elevation) * math.cos(azimuth) * y
            + math.cos(elevation) * z
        )
        return (u, v)

    @staticmethod
    def _polyline(points):
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in points)

    def _svg_axes(self, to_canvas):
        min_x = min(record["x"] for record in self.records)
        min_y = min(record["y"] for record in self.records)
        min_z = min(record["z"] for record in self.records)
        origin = (min_x - 0.015, min_y - 0.015, min_z - 0.015)
        axes = (
            ("X 0.05 m", "#dc2626", (0.05, 0.0, 0.0)),
            ("Y 0.05 m", "#16a34a", (0.0, 0.05, 0.0)),
            ("Z 0.05 m", "#2563eb", (0.0, 0.0, 0.05)),
        )
        lines = []
        ox, oy = to_canvas(self._project(*origin))
        for label, color, delta in axes:
            end = (
                origin[0] + delta[0],
                origin[1] + delta[1],
                origin[2] + delta[2],
            )
            ex, ey = to_canvas(self._project(*end))
            lines.append(
                f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" '
                f'stroke="{color}" stroke-width="1.5" />'
            )
            lines.append(
                f'<text x="{ex + 5:.1f}" y="{ey - 5:.1f}" '
                'font-family="Arial, sans-serif" font-size="12" '
                f'fill="{color}">{label}</text>'
            )
        return "".join(lines)

    @staticmethod
    def _escape(value):
        return (
            str(value)
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
        )
