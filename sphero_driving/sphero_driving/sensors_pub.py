import rclpy
from rclpy.node import Node

from sphero_interfaces.msg import Quaternion as QtMsg

from typing import Dict
from pysphero.core import Sphero
from pysphero.device_api.sensor import Quaternion

quaternion = [0.0, 0.0, 0.0, 0.0] # x, y, z, w


class QuaternionPublisher(Node):

  def __init__(self):
    super().__init__('quaternion_publisher') # Node name
    self.publisher_ = self.create_publisher(QtMsg, 'quaternion', 10)
    timer_period = 0.5
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    msg = QtMsg()
    msg.x = quaternion[0]
    msg.y = quaternion[1]
    msg.z = quaternion[2]
    msg.w = quaternion[3]
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing Quaternion... x: {}'.format(msg.x))


def notify_callback(data: Dict):
  global quaternion
  quaternion[0] = data.get(Quaternion.x)
  quaternion[1] = data.get(Quaternion.y)
  quaternion[2] = data.get(Quaternion.z)
  quaternion[3] = data.get(Quaternion.w)


def main(args=None):
  rclpy.init(args=args)
  quaternion_publisher = QuaternionPublisher()

  mac_address = "d6:bc:6a:05:79:b6"
  with Sphero(mac_address=mac_address) as sphero:
    sphero.power.wake()
    sphero.sensor.set_notify(notify_callback, Quaternion)

    rclpy.spin(quaternion_publisher)

    quaternion_publisher.destroy_node()
    rclpy.shutdown()

    sphero.sensor.cancel_notify_sensors()
    sphero.power.enter_soft_sleep()


if __name__ == '__main__':
  main()
