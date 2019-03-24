#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

class WayPoint:
	
	def __init__(self):

		rospy.init_node('ros_bridge')
	
		#rospy.init_node('pluto_fly', disable_signals = True)

		self.ros_bridge = cv_bridge.CvBridge()

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('visionSensor/image_rect', Image, self.image_callback)




 	
				
	def image_callback(self,msg):

		# 'image' is now an opencv frame
				# You can run opencv operations on 'image'
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		
		self.param1 = [150,0,0]
		self.param2 = [255,10,10]
		self.param3 = [0,0,200]
		self.param4 = [10,10,255]
		self.param5 = [0,200,0]
		self.param6 = [0,255,0]
		
		#rate = rospy.Rate(2)

		while (1):

			



			#cap = cv2.videocapture()
			#ret, frame = image.imread()
			#ret, imaga = image.imread()
			#ret, imagaa = image.imread()
		
			


			
			lower = np.array(self.param1)
			upper = np.array(self.param2)
			mask = cv2.inRange(image, lower, upper)
			ding, contours1, hierrachy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(mask,contours1, -1 ,(0,255,0), 3)
			k = len(contours1)
			print 'no of blues  /n', k

			pub5 = rospy.Publisher('/blue', Int32, queue_size = 10)
			pub5.publish(k)



			
			lower1 = np.array(self.param3)
			upper1 = np.array(self.param4)
			mask2 = cv2.inRange(image, lower1, upper1)
	

			ding, contours2, hierrachy = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			
			cv2.drawContours(mask2,contours2, -1 ,(0,255,0), 3)
			
			k2 = len(contours2)
			print 'no of red  %r', k2
			pub6 = rospy.Publisher('/red', Int32, queue_size = 10)
			pub6.publish(k2)



			
			lower2 = np.array(self.param5)
			upper2 = np.array(self.param6)
			mask3 = cv2.inRange(image, lower2,upper2)
			ding, contours3, hierrachy = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(mask3,contours3, -1 ,(0,255,0), 3)
			k3 = len(contours3)
			print 'no of greeen %r',  k3
			pub7 = rospy.Publisher('/green', Int32, queue_size = 10)
			pub7.publish(k3)






		cap.release()

		if cv2.waitKey(0) == 27:
			cv2.destoyAllWindows()
if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = WayPoint()
		rospy.spin()