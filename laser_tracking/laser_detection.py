import cv2
import numpy as np

def laser_detection(image):
	# image = cv2.resize(image,(1600,900))
	###hsv thresholding
	hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lowerb = np.array([100.*180./255., 100, 200])
	upperb = np.array([150.*180./255., 255, 255])
	laser_filtered = cv2.inRange(hsv, lowerb, upperb)

	###rgb thresholding
	# lowerb = np.array([0, 240, 0])
	# upperb = np.array([255, 255, 255])
	# laser_filtered = cv2.inRange(image, lowerb, upperb)


	# cv2.imshow('image',laser_filtered)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	count=[]
	ce=[]
	retval, labels, stats, centroids=cv2.connectedComponentsWithStats(laser_filtered)
	for i in range(len(stats)):
		if stats[i][4]>10 and stats[i][4]<5000:
			count.append(stats[i][4])
			ce.append(centroids[i])
			
	if sum(count)<30:
		raise ZeroDivisionError

	centroid=np.average(np.array(ce),axis=0,weights=np.array(count))

	return centroid

	

# laser_detection(cv2.imread("green.jpg"))

