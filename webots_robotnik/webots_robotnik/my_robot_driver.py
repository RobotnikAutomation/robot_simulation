import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.573
WHEEL_RADIUS = 0.127

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_front_motor = self.__robot.getDevice('robot_front_left_wheel_joint')
        self.__right_front_motor = self.__robot.getDevice('robot_front_right_wheel_joint')
        self.__left_rear_motor = self.__robot.getDevice('robot_back_left_wheel_joint')
        self.__right_rear_motor = self.__robot.getDevice('robot_back_right_wheel_joint')

        self.__left_front_motor.setPosition(float('inf'))
        self.__left_front_motor.setVelocity(0)
        
        self.__left_rear_motor.setPosition(float('inf'))
        self.__left_rear_motor.setVelocity(0)

        self.__right_front_motor.setPosition(float('inf'))
        self.__right_front_motor.setVelocity(0)
        
        self.__right_rear_motor.setPosition(float('inf'))
        self.__right_rear_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_front_motor.setVelocity(command_motor_left)
        self.__right_front_motor.setVelocity(command_motor_right)
        self.__left_rear_motor.setVelocity(command_motor_left)
        self.__right_rear_motor.setVelocity(command_motor_right)
