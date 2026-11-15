"""A scripted operator with the radio transmitter.

Publishes radio frames from its parameters, so a launch file or a test can
play the operator:

    ros2 param set /rc_transmitter mode auto      # mode switch (CH5)
    ros2 param set /rc_transmitter blade true     # blade switch (CH6)
    ros2 param set /rc_transmitter throttle 0.5   # sticks, -1 to 1
    ros2 param set /rc_transmitter steer -0.3     # positive turns left
    ros2 param set /rc_transmitter link false     # transmitter off or out of range

With auto_after >= 0 the operator switches to automatic mode and enables the
blade that many seconds after the first clock tick.
"""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import UInt16MultiArray

from . import topics
from .nodes import QOS_DEPTH, ModelNode
from .rc import unit_to_pulse

MODES = ('manual', 'auto')
NEVER = -1.0


def build_frame(rc, mode, blade, throttle, steer):
    """Transmitter frame for the stick and switch positions.

    The sticks are mixed in the transmitter, as on the robot today: the left
    side gets throttle - steer and the right side throttle + steer.
    """
    frame = [rc.pulse_mid] * rc.channel_count
    frame[rc.channels.left - 1] = unit_to_pulse(throttle - steer, rc)
    frame[rc.channels.right - 1] = unit_to_pulse(throttle + steer, rc)
    frame[rc.channels.mode - 1] = rc.pulse_max if mode == 'auto' else rc.pulse_min
    frame[rc.channels.blade - 1] = rc.pulse_max if blade else rc.pulse_min
    return frame


class TransmitterNode(ModelNode):

    def __init__(self):
        super().__init__('rc_transmitter')
        self.values = {
            'mode': self.declare_parameter('mode', 'manual').value,
            'blade': self.declare_parameter('blade', False).value,
            'throttle': self.declare_parameter('throttle', 0.0).value,
            'steer': self.declare_parameter('steer', 0.0).value,
            'link': self.declare_parameter('link', True).value,
        }
        self.auto_after = self.declare_parameter('auto_after', NEVER).value
        self.start = None
        self.add_on_set_parameters_callback(self.on_parameters)
        self.pub = self.create_publisher(UInt16MultiArray, topics.RC_TRANSMITTER, QOS_DEPTH)
        self.create_timer(self.electronics.rc.frame_period, self.step)

    def on_parameters(self, params):
        for p in params:
            if p.name == 'mode' and p.value not in MODES:
                return SetParametersResult(successful=False, reason=f'mode must be one of {MODES}')
            if p.name in self.values:
                self.values[p.name] = p.value
        return SetParametersResult(successful=True)

    def step(self):
        now = self.now()
        if self.start is None:
            self.start = now
        if self.auto_after >= 0 and now - self.start >= self.auto_after:
            self.values.update(mode='auto', blade=True)
            self.auto_after = NEVER
            self.get_logger().info('Operator: mode switch to automatic, blade switch on')
        if not self.values['link']:
            return
        v = self.values
        self.pub.publish(UInt16MultiArray(data=build_frame(
            self.electronics.rc, v['mode'], v['blade'], v['throttle'], v['steer'])))


def main(args=None):
    rclpy.init(args=args)
    node = TransmitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
