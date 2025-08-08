import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import String



class StationsMonitor(Node):

    def __init__(self):
        super().__init__('stations_monitor')

        self.publisher_ = self.create_publisher(String, 'station_status', 1)

        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        stations = [
            {'station_id': 'S1', 'x':  1.0, 'y':  1.0, 'item': 'A'},
            {'station_id': 'S2', 'x':  0.8, 'y':  0.3, 'item': 'B'},
            {'station_id': 'S3', 'x': -0.5, 'y': -0.6, 'item': 'C'},
            {'station_id': 'S4', 'x':  0.4, 'y':  1.3, 'item': 'D'},
            {'station_id': 'S5', 'x': -0.8, 'y':  0.6, 'item': '-'},
            {'station_id': 'S6', 'x':  0.5, 'y': -1.1, 'item': '-'},
        ]
    
        for i in range(len(stations)):
            msg = String()
            msg.data = json.dumps(stations[i])
            self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    stations_monitor = StationsMonitor()

    rclpy.spin(stations_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stations_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()