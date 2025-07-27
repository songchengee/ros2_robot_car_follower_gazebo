import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray

import numpy as np


class ImageListener(Node):
    def __init__(self, controller_node, sphere_node):
        super().__init__('image_listener')
        self.bridge = CvBridge()
        self.controller = controller_node
        self.sphere = sphere_node

        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.listener_callback,
            10)
        self.get_logger().info('Subscribed to /depth_camera/depth/image_raw')

    def listener_callback(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            off_center = self.compute_off_center(cv_depth)

            if off_center is not None:
                steer_angle = float(off_center * (-0.7854))  # scale factor, max ±45° in radians
                self.controller.set_steering(steer_angle, steer_angle)
                self.get_logger().info(f"Obstacle detected! Off-center: {off_center:.2f}")
                self.sphere.set_speed(4.0)
            else:
                self.controller.set_steering(0.0, 0.0)
                self.get_logger().info("No obstacle detected within threshold.")
                self.sphere.set_speed(0.0)
             

        except Exception as e:
            self.get_logger().error(f"Failed to process: {e}")

    def compute_off_center(self, cv_depth, min_thresh=1.0, max_thresh=8.0, bins=19):
        height, width = cv_depth.shape
        roi = cv_depth[height // 5 : height // 2, :]  # exclude bottom 50%

        obstacle_mask = (roi > min_thresh) & (roi < max_thresh)
        
        MIN_OBSTACLE_PIXELS = 1000  # adjust as needed
        #print(obstacle_mask)
        if np.count_nonzero(obstacle_mask) < MIN_OBSTACLE_PIXELS:
            return None


        bin_counts = np.zeros(bins)
        bin_width = width // bins

        for i in range(bins):
            col_start = i * bin_width
            col_end = (i + 1) * bin_width
            bin_counts[i] = np.count_nonzero(obstacle_mask[:, col_start:col_end])

        dominant_bin = np.argmax(bin_counts)
        center_bin = bins // 2
        off_center = (dominant_bin - center_bin) / center_bin  # normalized
        print(bin_counts)

        return off_center
            
class SteeringPublisher(Node):
    def __init__(self):
        super().__init__('steering_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/steering_controller/joint_trajectory',
            10)
        self.get_logger().info("Steering command node started.")

    def set_steering(self, left_angle, right_angle):
        msg = JointTrajectory()
        msg.joint_names = ['left_body_steer', 'right_body_steer']

        point = JointTrajectoryPoint()
        point.positions = [float(left_angle), float(right_angle)]
        point.time_from_start = Duration(sec=1)

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published steering command: {left_angle}, {right_angle}")
        
class SpherePublisher(Node):
    def __init__(self):
        super().__init__('sphere_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/sphere_controller/commands',
            10)

    def set_speed(self, speed):
        msg = Float64MultiArray()
        msg.data = [speed]
        self.publisher_.publish(msg)


        

def main(args=None):
    rclpy.init(args=args)

    steering_node = SteeringPublisher()
    sphere_node = SpherePublisher()
    image_node = ImageListener(steering_node, sphere_node)

    executor = MultiThreadedExecutor()
    executor.add_node(image_node)
    executor.add_node(steering_node)

    try:
        executor.spin()
    finally:
        image_node.destroy_node()
        steering_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
