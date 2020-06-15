import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from pysphero.core import Sphero
from pysphero.driving import Direction
from pysphero.utils import toy_scanner

class SimpleSpheroDriver(Node):

    def __init__(self, sphero):
        super().__init__('simple_sphero_driver')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.subscriber_callback, 10)
        self.subscription
        self.sphero = sphero

    def subscriber_callback(self, msg):
        speed = int(msg.linear.x * 100)
        heading = (int(msg.angular.z) + 3) * 60
        self.get_logger().info('speed: {0}, heading: {1}'.format(str(speed), str(heading)))
        self.sphero.driving.drive_with_heading(speed, heading, Direction.forward)



def main(args=None):

    mac_address = "d6:bc:6a:05:79:b6"
    with Sphero(mac_address=mac_address) as sphero:
        try:
            sphero.power.wake()
            rclpy.init(args=args)
            simple_sphero_driver = SimpleSpheroDriver(sphero)
            rclpy.spin(simple_sphero_driver)
            print("Spin end")
            sphero.power.enter_soft_sleep()

        except KeyboardInterrupt:
            print("Keyboarrd Interrupt!")
            pass

        finally:
            print("Finally")
            sphero.power.enter_soft_sleep()
            simple_sphero_driver.destroy_node()
            rclpy.shutdown()



if __name__ == "__main__":
    main()
