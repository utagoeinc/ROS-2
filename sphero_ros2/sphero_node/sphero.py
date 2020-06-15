#!/usr/bin/python

import rclpy
from rclpy.node import Node

import math
import sys
import time
from datetime import datetime
from datetime import timedelta
#import PyKDL

from sphero_driver import sphero_driver

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from sphero_msgs.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SpheroNode(Node):
    battery_state =  {1:"Battery Charging",
                      2:"Battery OK",
                      3:"Battery Low",
                      4:"Battery Critical"}


    ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]


    ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3]

    def __init__(self, default_update_rate=50.0):
        super().__init__('sphero')
        self.update_rate = default_update_rate
        self.sampling_divisor = int(400/self.update_rate)

        self.is_connected = False
        self._init_pubsub()
        self._init_params()
        self.robot = sphero_driver.Sphero()
        self.imu = Imu()
        self.imu.orientation_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.last_cmd_vel_time = datetime.now()
        self.last_diagnostics_time = datetime.now()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.power_state_msg = "No Battery Info"
        self.power_state = 0

    def _init_pubsub(self):
        self.odom_pub = self.create_publisher(Odometry, 'odom')
        self.imu_pub = self.create_publisher(Imu, 'imu')
        self.collision_pub = self.create_publisher(SpheroCollision, 'collision')
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics')
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel)
        self.color_sub = self.create_subscription(ColorRGBA, 'set_color', self.set_color)
        self.back_led_sub = self.create_subscription(Float32, 'set_back_led', self.set_back_led)
        self.stabilization_sub = self.create_subscription(Bool, 'disable_stabilization', self.set_stabilization)
        self.heading_sub = self.create_subscription(Float32, 'set_heading', self.set_heading)
        self.angular_velocity_sub = self.create_subscription(Float32, 'set_angular_velocity', self.set_angular_velocity)

    def _init_params(self):
        self.connect_color_red = 0 #rospy.get_param('~connect_red',0)
        self.connect_color_blue = 0 #rospy.get_param('~connect_blue',0)
        self.connect_color_green = 255 #rospy.get_param('~connect_green',255)
        self.cmd_vel_timeout = timedelta(seconds = 0.6) #rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.diag_update_rate = timedelta(seconds= 1.0) #rospy.Duration(rospy.get_param('~diag_update_rate', 1.0))

    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)

    def start(self):
        try:
            self.is_connected = self.robot.connect()
            self.get_logger().info("Connect to Sphero with address: {0}".format(self.robot.bt.target_address))
        except:
            self.get_logger().error("Failed to connect to Sphero.")
            sys.exit(1)
        #setup streaming
        self.get_logger().info("setup streaming")    
        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
        self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)
        #setup power notification
        self.get_logger().info("setup power notification")
        self.robot.set_power_notify(True, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['PWR_NOTIFY'], self.parse_power_notify)
        #setup collision detection
        self.get_logger().info("setup collision detection")
        self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)
        #set the ball to connection color
        self.get_logger().info("setup connection color")
        self.robot.set_rgb_led(self.connect_color_red,self.connect_color_green,self.connect_color_blue,0,False)
        #now start receiving packets
        self.get_logger().info("start")
        self.robot.start()
        self.timer = self.create_timer(0.1, self.spin)


    def spin(self):
        now = datetime.now()
        if (now-self.last_cmd_vel_time) > self.cmd_vel_timeout:
            # cmd timed out, set to 0
            if self.cmd_heading != 0 or self.cmd_speed != 0:
                self.get_logger().info("setting to 0")
                self.cmd_heading = 0
                self.cmd_speed = 0
                self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
            if (now-self.last_diagnostics_time) > self.diag_update_rate:
                self.last_diagnostics_time = now
                self.publish_diagnostics(now)
                    
    def stop(self):    
        #tell the ball to stop moving before quiting
        self.robot.roll(int(0), int(0), 1, False)
        self.robot.shutdown = True
        time.sleep(1.0)
        self.is_connected = self.robot.disconnect()
        self.robot.join()

    def publish_diagnostics(self, time):
        diag = DiagnosticArray()
        diag.header.stamp.sec = time.second
        diag.header.stamp.nanosec = time.microsecond * 1000
        
        stat = DiagnosticStatus(name="Battery Status", level=DiagnosticStatus.OK, message=self.power_state_msg)
        if self.power_state == 3:
            stat.level=DiagnosticStatus.WARN
        if self.power_state == 4:
            stat.level=DiagnosticStatus.ERROR
        diag.status.append(stat)

        self.diag_pub.publish(diag)


    def parse_collision(self, data):
        if self.is_connected:
            now = datetime.now()
            collision = SpheroCollision()
            collision.header.stamp.sec = now.second
            collision.header.stamp.nanosec = now.microsecond * 1000 
            collision.x = data["X"]
            collision.y = data["Y"]
            collision.z = data["Z"]
            collision.axis = int(data["Axis"])
            collision.x_magnitude = data["xMagnitude"]
            collision.y_magnitude = data["yMagnitude"]
            collision.speed = data["Speed"]
            collision.timestamp = data["Timestamp"]
            
            self.collision = collision
            self.collision_pub.publish(self.collision)
            

    def parse_power_notify(self, data):
        if self.is_connected:
            self.power_state = data
            self.power_state_msg = self.battery_state[data]

    def parse_data_strm(self, data):
        if self.is_connected:
            now = datetime.now()
            imu = Imu(header=std_msgs.Header(frame_id="imu_link"))
            imu.header.stamp.sec = now.second
            imu.header.stamp.nanosec = now.microsecond * 1000
            imu.orientation.x = data["QUATERNION_Q0"]
            imu.orientation.y = data["QUATERNION_Q1"]
            imu.orientation.z = data["QUATERNION_Q2"]
            imu.orientation.w = data["QUATERNION_Q3"]
            imu.linear_acceleration.x = data["ACCEL_X_FILTERED"]/4096.0*9.8
            imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"]/4096.0*9.8
            imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"]/4096.0*9.8
            imu.angular_velocity.x = data["GYRO_X_FILTERED"]*10*math.pi/180
            imu.angular_velocity.y = data["GYRO_Y_FILTERED"]*10*math.pi/180
            imu.angular_velocity.z = data["GYRO_Z_FILTERED"]*10*math.pi/180

            self.imu = imu
            self.imu_pub.publish(self.imu)

            odom = Odometry(header=std_msgs.Header(frame_id="odom"), child_frame_id='base_footprint')
            odom.header.stamp.sec = now.second
            odom.header.stamp.nanosec = now.microsecond * 1000
            odom.pose.pose = Pose(Point(data["ODOM_X"]/100.0,data["ODOM_Y"]/100.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            odom.twist.twist = Twist(Vector3(data["VELOCITY_X"]/1000.0, 0, 0), Vector3(0, 0, data["GYRO_Z_FILTERED"]*10.0*math.pi/180.0))
            odom.pose.covariance =self.ODOM_POSE_COVARIANCE                
            odom.twist.covariance =self.ODOM_TWIST_COVARIANCE
            self.odom_pub.publish(odom)                      

            # #need to publish this trasform to show the roll, pitch, and yaw properly
            # self.transform_broadcaster.sendTransform((0.0, 0.0, 0.038 ),
            #     (data["QUATERNION_Q0"], data["QUATERNION_Q1"], data["QUATERNION_Q2"], data["QUATERNION_Q3"]),
            #     odom.header.stamp, "base_link", "base_footprint")

    def cmd_vel(self, msg):
        if self.is_connected:
            self.last_cmd_vel_time = datetime.now()
            self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
            self.cmd_speed = math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))
            self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
    
    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg.r*255),int(msg.g*255),int(msg.b*255),0,False)

    def set_back_led(self, msg):
        if self.is_connected:
            self.robot.set_back(msg.data, False)

    def set_stabilization(self, msg):
        if self.is_connected:
            if not msg.data:
                self.robot.set_stablization(1, False)
            else:
                self.robot.set_stablization(0, False)

    def set_heading(self, msg):
        if self.is_connected:
            heading_deg = int(self.normalize_angle_positive(msg.data)*180.0/math.pi)
            self.robot.set_heading(heading_deg, False)

    def set_angular_velocity(self, msg):
        self.get_logger().info("in set_angular_velocity")
        if self.is_connected:
            self.get_logger().info("connected")
            rate = int((msg.data*180/math.pi)/0.784)
            self.robot.set_rotation_rate(rate, False)
            self.get_logger().info('rotation rate set')

    def configure_collision_detect(self, msg):
        pass

    # def reconfigure(self, config, level):
    #     if self.is_connected:
    #         self.robot.set_rgb_led(int(config['red']*255),int(config['green']*255),int(config['blue']*255),0,False)
    #     return config

def main(args=None):
    rclpy.init(args=args)
    s = None
    try:
        s = SpheroNode()
        s.start()
        rclpy.spin(s)
        s.stop()
    finally:
        if s:
            s.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()

