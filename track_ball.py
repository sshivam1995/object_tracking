#!/usr/bin/env python3

# import needed items
import numpy as np
import cv2
import rospy
import geometry_msgs.msg
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan


class BallTracker:
    def __init__(self):
        # set HSV limits for mask
        # self.pinkLower = (150, 100, 100)
        # self.pinkUpper = (180, 255, 255)
        self.yellowLower = (20, 100, 100)
        self.yellowUpper = (80, 255, 255)
        self.center_pt = geometry_msgs.msg.Point(-1, -1, -1)
        self.obj_position = geometry_msgs.msg.Point(180, -1, -1)
        self.object_dist = -1
        self.obj_angle = 180
        self.obj_span = 0
        print("in constructor")
        # make kernel
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self.image_subscriber = rospy.Subscriber(
            "/raspicam_node/image/compressed", CompressedImage, self.callback_image_received)
        self.lidar_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.callback_lidar_dist_received)
        self.center_publisher = rospy.Publisher(
            "/center_pt", geometry_msgs.msg.Point, queue_size=1)
        self.output_pub = rospy.Publisher(
            "/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.mask_pub = rospy.Publisher(
            "/mask/image_raw/compressed", CompressedImage, queue_size=1)

    def callback_image_received(self, msg):
        print("img received")
        self.img_data = msg.data
        self.process_image()
        self.obj_position.z = self.object_dist
        self.obj_position.x = self.obj_angle
        self.center_publisher.publish(self.obj_position)

    def callback_lidar_dist_received(self, msg):
        min_index = np.floor(self.obj_angle - self.obj_span/2).astype(int)
        max_index = np.floor(self.obj_angle + self.obj_span/2).astype(int)
        count = 0.01
        distance = 0
        if max_index < 0:
            min_index = 360 + min_index
            max_index = 360 + max_index
            for index in range(min_index, max_index+1):
                if msg.ranges[index] != 0:
                    distance = distance+msg.ranges[index]
                    count = count + 1
            distance = distance/count
        elif min_index < 0 and max_index > 0:
            min_index = 360 + min_index
            for index in range(min_index, 360):
                if msg.ranges[index] != 0:
                    distance = distance+msg.ranges[index]
                    count = count + 1
            for index in range(0, max_index):
                if msg.ranges[index] != 0:
                    distance = distance+msg.ranges[index]
                    count = count + 1
            distance = distance/count
        else:
            for index in range(min_index, max_index+1):
                if msg.ranges[index] != 0:
                    distance = distance+msg.ranges[index]
                    count = count + 1
            distance = distance/count
        self.object_dist = distance

    def process_image(self):
        np_arr = np.fromstring(self.img_data, np.uint8)
        cvImage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_width = np.size(cvImage, 1)
        self.frame = cvImage
        self.frame_output = self.frame.copy()
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # create mask with making elements in range white
        self.mask = cv2.inRange(hsv, self.yellowLower, self.yellowUpper)
        # remove noise from mask and fill holes using close and open
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, self.kernel)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, self.kernel)

        mask_image = CompressedImage()
        mask_image.header.stamp = rospy.Time.now()
        mask_image.format = "jpeg"
        mask_image.data = np.array(
            cv2.imencode('.jpg', self.mask)[1]).tostring()
        self.mask_pub.publish(mask_image)

        # get bounding circle of mask
        self.capture_outline()

    def capture_outline(self):
        # show created mask
        # use findContours to get contour of white in mask
        trash, contours, hierarchy = cv2.findContours(self.mask, cv2.RETR_EXTERNAL,
                                                      cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea,
                          reverse=True)[:1]
        self.center_pt.x = -1
        self.center_pt.y = -1
        self.center_pt.z = self.image_width
        print("contour length= ", len(contours))
        if len(contours) > 0:
            # if contour is found
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]),
                      int(M["m01"] / M["m00"]))
            # draw the circle in the output image and mark center
            if radius > 20:
                self.obj_radius = radius
                self.center_pt.x = x
                self.center_pt.y = y
                # only do for large objects to remove false positives
                print("Circle found at ",
                      center[0], ", ", center[1], ", ", radius)
                cv2.circle(self.frame_output, center,
                           int(radius), (0, 255, 0), 4)
                cv2.rectangle(self.frame_output, (int(x - 5), int(y - 5)),
                              (int(x + 5), int(y + 5)), (0, 128, 255), -1)
                output_image = CompressedImage()
                output_image.header.stamp = rospy.Time.now()
                output_image.format = "jpeg"
                output_image.data = np.array(
                    cv2.imencode('.jpg', self.frame_output)[1]).tostring()
                self.output_pub.publish(output_image)
        self.get_angle_of_object()

    def get_angle_of_object(self):
        self.obj_angle = 180
        self.obj_span = 0
        if self.center_pt.x > 0 and self.center_pt.y > 0:
            self.obj_angle = np.floor(
                ((self.center_pt.x-240)/480)*62.2).astype(int)
            self.obj_span = np.floor(
                (self.obj_radius/480)*62.2).astype(int)


if __name__ == '__main__':
    rospy.init_node('ball_tracker')
    BallTracker()
    rospy.spin()
    cv2.destroyAllWindows()
