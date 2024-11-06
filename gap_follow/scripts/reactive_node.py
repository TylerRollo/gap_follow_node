#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'

        # Create a subscriber to LIDAR data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            self.lidarscan_topic,
            self.lidar_callback,
            10
        )

        # Create a publisher for driving commands
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

    """ Preprocess the LiDAR scan array. 
          1. Setting each value to the mean over some window
          2. Rejecting high values (eg. > 3m)
    """
    def preprocess_lidar(self, ranges):
        return np.where(np.array(ranges) > 3.0, np.nan, ranges)
    
    """ Return the start index & end index of the max gap in free_space_ranges"""
    def find_max_gap(self, free_space_ranges):
        max_gap_start, max_gap_end = -1, -1
        current_gap_start = -1
        max_gap_length = 0

        for i, value in enumerate(free_space_ranges):
            if value > 0:
                if current_gap_start == -1:
                    current_gap_start = i  # Start a new gap
            else:
                if current_gap_start != -1:
                    # End of the current gap
                    gap_length = i - current_gap_start
                    if gap_length > max_gap_length:
                        max_gap_length = gap_length
                        max_gap_start, max_gap_end = current_gap_start, i - 1
                    current_gap_start = -1  # Reset the current gap start

        # Handle the case where the last gap might be the longest
        if current_gap_start != -1:
            gap_length = len(free_space_ranges) - current_gap_start
            if gap_length > max_gap_length:
                max_gap_start, max_gap_end = current_gap_start, len(free_space_ranges) - 1

        return max_gap_start, max_gap_end

    """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
    """
    def find_best_point(self, start_i, end_i, ranges):
        if start_i != -1 and end_i != -1 and start_i < end_i:
            # Find the index of the maximum value in the range
            best_index = start_i + np.argmax(ranges[start_i:end_i])
            return best_index
        return None

    def get_speed(self, steering_angle):
        """ Estimate speed based on steering angle
        """
        if abs(steering_angle) >= np.radians(0) and abs(steering_angle) < np.radians(10):
            return 3.0
        elif abs(steering_angle) >= np.radians(10) and abs(steering_angle) < np.radians(20):
            return 1.0
        else:
            return 0.5        

    """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message. """
    def lidar_callback(self, data):
        proc_ranges = self.preprocess_lidar(data.ranges)
        
        # TODO:
        #Find closest point to LiDAR
        closest_index = np.nanargmin(data.ranges)
        closest_distance = proc_ranges[closest_index]

        #Eliminate all points inside 'bubble' (set them to zero)
        bubble_radius = .75
        bubble_indicies = int(0.3 / 0.01)
        
        start_index = int(max(0, closest_index - bubble_indicies))
        end_index = int(min(len(proc_ranges), closest_index + bubble_radius + 1))
       
        proc_ranges[start_index:end_index] = [0] * (end_index - start_index)

        #Find max length gap 
        max_gap_start, max_gap_end = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_point_index = self.find_best_point(max_gap_start, max_gap_end, proc_ranges)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        if best_point_index is not None:
            # Set target speed and steering angle towards the best point
            
            drive_msg.drive.steering_angle = np.arctan2(
                proc_ranges[best_point_index] * np.sin(best_point_index * 0.01),  # Convert index to angle
                proc_ranges[best_point_index] * np.cos(best_point_index * 0.01)
            )
            drive_msg.drive.speed = self.get_speed(drive_msg.drive.steering_angle)  # Set desired speed
        else:
            drive_msg.drive.speed = 0.0  # Stop if no valid point is found
            drive_msg.drive.steering_angle = 0.0
        
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
