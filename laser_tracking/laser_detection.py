import cv2
import numpy as np

def laser_detection(image):
	# image = cv2.resize(image,(1600,900))
	hsv=cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
	lowerb = np.array([180*179/255, 50, 200])
	upperb = np.array([255*179/255, 255, 255])
	laser_filtered = cv2.inRange(hsv, lowerb, upperb)
	# cv2.imshow('image',laser_filtered)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	retval, labels, stats, centroids=cv2.connectedComponentsWithStats(laser_filtered)
	for i in range(len(stats)):
		if stats[i][4]>30 and stats[i][4]<5000:
			# print(stats[i][4])
			return(centroids[i])

# laser_detection(cv2.imread("test/test5.jpg"))

