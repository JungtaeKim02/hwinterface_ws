#!/usr/bin/env python3


import math
import time
import tkinter as tk

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


class KeyboardTeleop(Node):
    MOTION_KEYS = frozenset(('up', 'down', 'left', 'right', 'q', 'e'))
    SHIFT_KEYS = frozenset(('shift_l', 'shift_r'))

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed_x', 0.8)
        self.declare_parameter('linear_speed_y', 0.25)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('uart_enabled', False)
        self.speed_x = float(self.get_parameter('linear_speed_x').value)
        self.speed_y = float(self.get_parameter('linear_speed_y').value)
        self.speed_yaw = float(self.get_parameter('angular_speed').value)
        if not all(math.isfinite(value) and value > 0
                   for value in (self.speed_x, self.speed_y, self.speed_yaw)):
            raise ValueError('Keyboard speeds must be finite and positive')
        self.uart_enabled = bool(self.get_parameter('uart_enabled').value)
        self.publisher = self.create_publisher(TwistStamped, '/omni_drive/cmd_vel', 1)
        self.create_subscription(DiagnosticArray, '/uart/drive_status', self.receive_status, 1)
        self.arm_client = self.create_client(Trigger, '/uart/arm')
        self.disarm_client = self.create_client(Trigger, '/uart/disarm')
        self.keys = set()
        self.release_callbacks = {}
        self.focused = False
        self.inhibited = False
        self.drive_ready = not self.uart_enabled
        self.local_disarm = False
        self.arm_pending = False
        self.status_received = 0.0
        self.request_future = None
        self.notice = ''
        self.drive_status_text = 'Waiting for MCU state' if self.uart_enabled else 'Simulation'
        self.running = True
        self.window = tk.Tk()
        self.window.title('RoboCup keyboard control')
        self.window.geometry('590x340')
        self.window.configure(padx=20, pady=16)
        tk.Label(self.window, text='Hold Shift + a motion key', font=('', 17, 'bold')).pack()
        tk.Label(self.window, justify='left', text=(
            '↑ / ↓     Forward / backward\n'
            '← / →     Left / right\n'
            'Q / E      Counterclockwise / clockwise\n'
            'Space      Zero velocity, keep drive enabled\n'
            'Esc         DISARM\n'
            'A            ARM after normal DISARM\n'
            'Ctrl+C     Close and DISARM')).pack(pady=12)
        tk.Label(self.window, text=(
            f'Forward {self.speed_x:g} m/s  ·  Lateral {self.speed_y:g} m/s  ·  '
            f'Yaw {self.speed_yaw:g} rad/s')).pack()
        self.state_text = tk.StringVar(value='Click this window to control')
        tk.Label(self.window, textvariable=self.state_text, wraplength=545).pack(pady=12)
        self.window.bind('<KeyPress>', self.press)
        self.window.bind('<KeyRelease>', self.release)
        self.window.bind('<FocusIn>', self.focus_in)
        self.window.bind('<FocusOut>', self.focus_out)
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.after(0, self.tick)

    def focus_in(self, _event):
        self.focused = True

    def focus_out(self, _event):
        self.focused = False
        self.keys.clear()
        self.publish_velocity()

    def press(self, event):
        key = event.keysym.lower()
        release_callback = self.release_callbacks.pop(key, None)
        if release_callback is not None:
            self.window.after_cancel(release_callback)
        repeated = key in self.keys
        self.keys.add(key)
        if key == 'c' and event.state & 0x4:
            self.close()
            return
        if repeated:
            return
        if key == 'space':
            self.inhibited = True
            self.publish_velocity()
        elif key == 'escape':
            self.local_disarm = True
            self.arm_pending = False
            self.publish_velocity()
            self.request_control('DISARM')
        elif key == 'a':
            self.publish_zero()
            if self.keys & (self.MOTION_KEYS | self.SHIFT_KEYS):
                self.notice = 'Release Shift and motion keys before ARM'
            else:
                self.request_control('ARM')
        elif key in self.MOTION_KEYS or key in self.SHIFT_KEYS:
            self.publish_velocity()

    def release(self, event):

        key = event.keysym.lower()
        previous = self.release_callbacks.pop(key, None)
        if previous is not None:
            self.window.after_cancel(previous)
        self.release_callbacks[key] = self.window.after_idle(self.finish_release, key)

    def finish_release(self, key):
        self.release_callbacks.pop(key, None)
        self.keys.discard(key)
        if not self.keys & (self.MOTION_KEYS | self.SHIFT_KEYS):
            self.inhibited = False
        self.publish_velocity()

    def receive_status(self, message):
        for status in message.status:
            if status.name != 'robocup/uart_drive':
                continue
            values = {value.key: value.value for value in status.values}
            self.status_received = time.monotonic()
            self.drive_ready = (values.get('drive_state') == 'ARMED'
                                and values.get('state_fresh') == 'true'
                                and values.get('disarm_latched') == 'false')
            if self.arm_pending and self.drive_ready:
                self.local_disarm = False
                self.arm_pending = False
            self.drive_status_text = f"MCU {values.get('drive_state', 'UNKNOWN')} · {status.message}"

    def request_control(self, action):
        self.publish_zero()
        if not self.uart_enabled:
            self.local_disarm = action == 'DISARM'
            self.notice = f'Simulation {action}'
            return
        client = self.disarm_client if action == 'DISARM' else self.arm_client
        if not client.service_is_ready():
            self.notice = f'/uart/{action.lower()} service unavailable'
            return
        if self.request_future is not None and not self.request_future.done():
            if action == 'ARM':
                self.notice = 'Wait for the current control request'
                return
            self.request_future.cancel()
        self.arm_pending = action == 'ARM'
        self.request_future = client.call_async(Trigger.Request())
        self.request_future.add_done_callback(self.control_done)
        self.notice = f'{action} requested'

    def control_done(self, future):
        if future.cancelled():
            return
        try:
            response = future.result()
            self.notice = response.message
            if not response.success:
                self.arm_pending = False
        except Exception as error:
            self.arm_pending = False
            self.notice = str(error)

    def publish_zero(self):
        self.publish_velocity(force_zero=True)

    def publish_velocity(self, force_zero=False):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        fresh = not self.uart_enabled or time.monotonic() - self.status_received < 0.5
        enabled = (not force_zero and self.focused and not self.inhibited
                   and not self.local_disarm and not self.arm_pending
                   and self.drive_ready and fresh and bool(self.keys & self.SHIFT_KEYS))
        if enabled:
            message.twist.linear.x = self.speed_x * (('up' in self.keys) - ('down' in self.keys))
            message.twist.linear.y = self.speed_y * (('left' in self.keys) - ('right' in self.keys))
            message.twist.angular.z = self.speed_yaw * (('q' in self.keys) - ('e' in self.keys))
        self.publisher.publish(message)

    def tick(self):
        if not self.running:
            return
        if not rclpy.ok():
            self.window.quit()
            return
        rclpy.spin_once(self, timeout_sec=0)
        self.publish_velocity()
        focus = 'Focused' if self.focused else 'Unfocused — zero velocity'
        self.state_text.set(f'{focus} · {self.drive_status_text}\n{self.notice}')
        self.window.after(50, self.tick)

    def close(self):
        self.running = False
        self.local_disarm = True
        if rclpy.ok():
            self.publish_zero()
            self.request_control('DISARM')
        self.window.quit()

    def run(self):
        try:
            self.window.mainloop()
        finally:
            if rclpy.ok():
                self.publish_zero()
                if self.uart_enabled and self.disarm_client.service_is_ready():
                    future = self.disarm_client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.3)
            self.window.destroy()


def main():
    rclpy.init()
    node = None
    try:
        node = KeyboardTeleop()
        node.run()
    except (KeyboardInterrupt, tk.TclError) as error:
        if node is not None:
            node.get_logger().info(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
