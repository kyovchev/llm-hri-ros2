import rclpy
from std_msgs.msg import String
import math
import json


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class RobotDriver:
    def init(self, webots_node, parameters):
        parameters

        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__id = self.__robot.getName()

        self.__gps = self.__robot.getDevice('gps')
        self.__compass = self.__robot.getDevice('compass')
        self.__gps.enable(16)
        self.__compass.enable(16)

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__pickup = None
        self.__drop_off = None

        rclpy.init(args=None)
        self.__node = rclpy.create_node(f'{self.__id}_driver')
        self.__node.create_subscription(
            String, 'cmd_pos', self.__cmd_pos_callback, 1)

        self.__status = None
        self.__status_publisher = self.__node.create_publisher(
            String, 'robot_status', 1)

        self.__home = None

        timer_period = 3  # seconds
        self.timer = self.__node.create_timer(
            timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.__status:
            msg = String()
            msg.data = json.dumps(self.__status)
            self.__status_publisher.publish(msg)

    def __cmd_pos_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if self.__id == data['robot']:
                self.__pickup = data['item_position']
                self.__drop_off = data['delivery_station_position']
        except:
            self.__node.get_logger().error(f'Data error: {msg.data}')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        coords = self.__gps.getValues()
        north = self.__compass.getValues()
        rad = math.atan2(north[1], north[0]) + 2 * math.pi

        if not self.__home:
            self.__home = [coords[0], coords[1]]

        pos = self.__pickup or self.__drop_off

        go_to_home = False
        if not pos:
            go_to_home = True
            pos = self.__home

        forward_speed = 0
        angular_speed = 0

        try:
            if pos:
                drad = math.atan2(pos[0] - coords[0],
                                  pos[1] - coords[1]) + 2 * math.pi

                if drad > rad + 0.02:
                    angular_speed = -0.15
                elif drad < rad - 0.02:
                    angular_speed = 0.15
                elif math.hypot(pos[1] - coords[1], pos[0] - coords[0]) > 0.01:
                    forward_speed = 0.1
                else:
                    if self.__pickup:
                        self.__pickup = None
                    else:
                        self.__drop_off = None

                command_motor_left = (
                    forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
                command_motor_right = (
                    forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

                self.__left_motor.setVelocity(command_motor_left)
                self.__right_motor.setVelocity(command_motor_right)
            else:
                self.__node.get_logger().error(
                    f'Error with pickup: {self.__pickup} or drop_off: {self.__drop_off}')
                self.__pickup = None
                self.__drop_off = None
        except:
            pass

        status = {
            'robot_id': self.__id,
            'x': coords[0],
            'y': coords[1],
            'available': (forward_speed == 0 and angular_speed == 0) or go_to_home
        }

        if not self.__status or abs(self.__status['x'] - status['x']) > 0.005 or abs(self.__status['y'] - status['y']) > 0.005 or self.__status['available'] != status['available']:
            self.__status = status

            msg = String()
            msg.data = json.dumps(status)
            self.__status_publisher.publish(msg)
