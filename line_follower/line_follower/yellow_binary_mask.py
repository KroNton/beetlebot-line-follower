import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2

import numpy as np


class ImageSubscriber(Node):

    def __init__(self):

        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(

            Image,

            '/camera/image_raw',

            self.listener_callback,

            10)

        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()



    def listener_callback(self, data):

        # Convert ROS Image message to OpenCV image

        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        

        # Convert BGR to HSV

        hsv_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)



        # Define range of yellow color in HSV

        # Lower limit for yellow in HSV
        lower_yellow = np.array([20, 100, 100])

        # Upper limit for yellow in HSV
        upper_yellow = np.array([30, 255, 255])


        # Create a binary mask

        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)



        # Apply the mask to the original image

        yellow_segmented_image = cv2.bitwise_and(current_frame, current_frame, mask=yellow_mask)


        # Display the segmented image

        cv2.imshow("yellow Segmented Image", yellow_segmented_image)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()