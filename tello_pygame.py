#!/usr/bin/env python2

from djitellopy import Tello
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time
import rospy
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from darknet_ros_msgs.msg import BoundingBoxes

# Speed of the drone
S = 20
# Frames per second of the pygame window display
FPS = 25


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
            - F: Autonomous follow
            - M: Manual (default)

    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        # Object varaiables
        self.x = 0 
        self.y = 0
        self.h = 0 
        self.w = 0
        self.area = 0.0
        self.auto_follow = False

        # Status
        self.takeoff_status = False
        self.land_status = False

        self.send_rc_control = False
        self.should_stop = False
        self.image_publisher = rospy.Publisher('/tello/image', Image, queue_size=1)
        self.takeoff_msg = rospy.Subscriber('/tello/takeoff', Bool, self.takeoff_callback, queue_size=1)
        self.land_msg = rospy.Subscriber('/tello/land', Bool, self.land_callback, queue_size=1)
        self.detector_msg = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_callback, queue_size=1)
        self.bridge = CvBridge()
        # create update timer
        pygame.time.set_timer(USEREVENT + 1, 50)
	self.thread = threading.Thread(target=self.run)
	self.thread.start()

    def takeoff_callback(self, data):
        self.takeoff_status = data
        if data:
            self.land_status = False

    def land_callback(self, data):
        self.land_status = data
        if data:
            self.takeoff_status = False

    def boxes_callback(self, data):
	boxes = len(data.bounding_boxes)
	self.w = 0
	self.h = 0
	for i in range(boxes):
		if (data.bounding_boxes[i].Class == "sports ball"):
			self.x = data.bounding_boxes[i].xmin
			self.y = data.bounding_boxes[i].ymin
			self.w = data.bounding_boxes[i].xmax
			self.h = data.bounding_boxes[i].ymax
			#received_marker = True
			#print("Marker:",received_marker)

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        self.should_stop = False
        while not self.should_stop:

            for event in pygame.event.get():
                if event.type == USEREVENT + 1:
                    self.update()
                elif event.type == QUIT:
                    self.should_stop = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                frame_read.stop()
                break

            if self.auto_follow and self.area > 0.0:
                if self.area < 35 and self.area > 25: #change area threshold according to sensitivity
                    self.for_back_velocity = S
                    self.update()
                    #time.sleep(0.1)
                    self.for_back_velocity = 0
                    self.update()
                elif (self.area > 40 or self.area < 25):
                    #self.for_back_velocity = -S
                    #self.update()
                    self.for_back_velocity = 0
                    self.update()

                if (self.x < (960/3)): #change yaw threshold
                    self.yaw_velocity = -60
                    self.update()
                    #time.sleep(1)
                    self.yaw_velocity = 0
                    self.update()
                elif (self.x > (2*(960/3))):
                    self.yaw_velocity = 60
                    self.update()
                    #time.sleep(1)
                    self.yaw_velocity = 0
                    self.update()
            self.screen.fill([0, 0, 0])
            frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)

            #### Face detection ####
            image = frame
            frame = cv2.rectangle(image,(self.x,self.y),(self.w,self.h),(0,0,255),4)
	    self.area = ((self.w-self.x)*(self.h-self.y)/1000)
            #print("Area: ", self.area)
            frame = np.rot90(frame)
            frame = np.flipud(frame)
            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))
            time.sleep(1 / FPS)

        # Call it always before finishing. I deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw counter clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False
        elif key == pygame.K_f:  # autonomous
            print("Switching to autonomous")
            self.auto_follow = True
        elif key == pygame.K_m:  # manual
            print("Switching to manual")
            self.auto_follow = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)


def main():
    rospy.init_node('Tello_node',anonymous=True)
    frontend = FrontEnd()
    while not rospy.is_shutdown():
        rospy.spin()
    frontend.should_stop = True
    # run frontend
    # frontend.run()


if __name__ == '__main__':
    main()
