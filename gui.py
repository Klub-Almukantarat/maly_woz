#!/usr/bin/env python3
import gi
import subprocess
import signal
import os

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk


class Ros2Gui(Gtk.Window):
    def __init__(self):
        super().__init__(title="ROS2 Teleop Controller")
        self.set_default_size(600, 400)
        self.fullscreen()

        self.proc = None  # handle for the ros2 launch process

        # Layout
        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=20)
        self.add(vbox)

        # --- Control buttons ---
        btn_box = Gtk.Box(spacing=20)
        start_btn = Gtk.Button(label="Start ROS2 Launch")
        stop_btn = Gtk.Button(label="Stop ROS2 Launch")

        start_btn.connect("clicked", self.on_start_clicked)
        stop_btn.connect("clicked", self.on_stop_clicked)

        btn_box.pack_start(start_btn, True, True, 0)
        btn_box.pack_start(stop_btn, True, True, 0)
        vbox.pack_start(btn_box, False, False, 10)

        # --- Parameter sliders ---
        frame = Gtk.Frame(label="Teleop Parameters")
        grid = Gtk.Grid(column_spacing=10, row_spacing=10, margin=10)

        # Linear velocity (x)
        self.lin_adj = Gtk.Adjustment(0.5, 0.0, 3.0, 0.1, 0.5, 0)
        self.lin_slider = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL, adjustment=self.lin_adj
        )
        self.lin_slider.set_digits(2)
        self.lin_slider.connect("value-changed", self.on_lin_changed)

        # Angular velocity (yaw)
        self.yaw_adj = Gtk.Adjustment(0.5, 0.0, 3.0, 0.1, 0.5, 0)
        self.yaw_slider = Gtk.Scale(
            orientation=Gtk.Orientation.HORIZONTAL, adjustment=self.yaw_adj
        )
        self.yaw_slider.set_digits(2)
        self.yaw_slider.connect("value-changed", self.on_yaw_changed)

        grid.attach(Gtk.Label(label="Linear X:"), 0, 0, 1, 1)
        grid.attach(self.lin_slider, 1, 0, 1, 1)
        grid.attach(Gtk.Label(label="Yaw:"), 0, 1, 1, 1)
        grid.attach(self.yaw_slider, 1, 1, 1, 1)

        frame.add(grid)
        vbox.pack_start(frame, True, True, 10)

    # --- Button actions ---
    def on_start_clicked(self, widget):
        if self.proc is None:
            print("Starting ROS2 launch...")
            # Example: run ros2 launch inside docker
            self.proc = subprocess.Popen(
                [
                    "/home/ubuntu/almu/maly_woz/start-joy.sh",
                ]
            )
        else:
            print("Process already running.")

    def on_stop_clicked(self, widget):
        if self.proc:
            print("Stopping ROS2 launch...")
            self.proc.send_signal(signal.SIGINT)
            self.proc = None
        else:
            print("No process running.")

    # --- Parameter change handlers ---
    def on_lin_changed(self, slider):
        value = slider.get_value()
        print(f"Setting linear velocity to {value}")
        subprocess.Popen(
            [
                "docker",
                "exec",
                "maly_woz_hw",
                "bash",
                "-c",
                f"ros2 param set /teleop_node linear_x {value}",
            ]
        )

    def on_yaw_changed(self, slider):
        value = slider.get_value()
        print(f"Setting yaw velocity to {value}")
        subprocess.Popen(
            [
                "docker",
                "exec",
                "maly_woz_hw",
                "bash",
                "-c",
                f"ros2 param set /teleop_node linear_x {value}",
            ]
        )

    def on_yaw_changed(self, slider):
        value = slider.get_value()
        print(f"Setting yaw velocity to {value}")
        subprocess.Popen(
            [
                "docker",
                "exec",
                "maly_woz_hw",
                "ros2",
                "param",
                "set",
                "/teleop_node",
                "yaw",
                str(value),
            ]
        )


def main():
    win = Ros2Gui()
    win.connect("destroy", Gtk.main_quit)
    win.show_all()
    Gtk.main()


if __name__ == "__main__":
    main()
