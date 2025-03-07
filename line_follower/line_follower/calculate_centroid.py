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

        line_centroid = self.get_contour_data(yellow_mask,current_frame)
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(current_frame, contours, -1, (0, 255, 0), 2)  # Green color, thickn
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(current_frame, [largest_contour], -1, (255, 0, 0), 2)  # Blue color, thickness=2

        # Display the segmented image with line centroid

        if line_centroid:

            cv2.circle(yellow_segmented_image, (line_centroid['x'], line_centroid['y']), 5, (0, 0, 255), 7)

        # Display the segmented image

        cv2.imshow("yellow Segmented Image", yellow_segmented_image)

        cv2.waitKey(1)


    def get_contour_data(self, mask,current_frame):

        """

        Return the centroid of the largest contour in the binary image 'mask' (the line) 

        """ 

        # Constants

        MIN_AREA_TRACK = 10000  # Minimum area for track marks



        # get a list of contours

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


        line = {}


        for contour in contours:

            M = cv2.moments(contour)


            if (M['m00'] > MIN_AREA_TRACK):

                # Contour is part of the track

                line['x'] = int(M["m10"]/M["m00"])

                line['y'] = int(M["m01"]/M["m00"])



        return (line)

def main(args=None):

    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()
            # Detect line and get its centroid

    
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()