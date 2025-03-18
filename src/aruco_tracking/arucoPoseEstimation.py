import numpy as np
import cv2
import sys
import time
from cv_bridge import CvBridge
import pickle



ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


class ArucoPoseEstimation():

	def __init__(self):
		#self.__init__("aruco_pose_estimation")

		self.aruco_type = "DICT_6X6_250"  #FOR SIM "DICT_6X6_250"

		self.bridge = CvBridge()

		self.arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[self.aruco_type])

		self.arucoParams = cv2.aruco.DetectorParameters()

		self.is_detected = False

		self.tvec_array = []
		self.rvec_array = []

		#REAL CAMERA MATRIX
		#self.intrinsic_camera = np.array(((676.55834453 , 0.0 , 296.84316385),(0.0 , 678.23503143 , 247.96239129),(0.0 , 0.0 , 1.0)))
		#self.distortion  = np.array((-0.42625153 , 0.15396174 , -0.00297538 , 0.00110313 , 0.10237866))
		
		#GAZEBO CAMERA MATRIX
		self.intrinsic_camera = np.array(((528.433756558705, 0, 320.5),(0,528.433756558705, 240.5),(0,0,1)))
		self.distortion = np.array((-0.25,0.12,-0.00028,-0.00005,0))

		
   

		#read calibration file
		#pickle.dump((self.intrinsic_camera, self.distortion), open( "src/aruco_tracking/config_file/cameraMatrix.pkl", "wb" ))
		#pickle.dump(self.intrinsic_camera, open( "/home/morolinux/Documents/sbem_project_ws/src/aruco_tracking/config_file/cameraMatrix.pkl", "wb" ))
		#pickle.dump(self.distortion, open( "/home/morolinux/Documents/sbem_project_ws/src/aruco_tracking/config_file/dist.pkl", "wb" ))


	def aruco_process_image(self,image):
		
		frame = self.bridge.imgmsg_to_cv2(image,"bgr8")

		#frame = cv2.VideoCapture(0)
		frame = cv2.resize(frame,(640,480))



		#if frame.isOpened():

		#ret, img = frame.read()
		output = self.pose_estimation(frame, ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
		

		cv2.imshow('Estimated Pose', output)
		key = cv2.waitKey(1) & 0xFF
		#if key == ord('q'):
		#	break

		

	
	def close_windows(self,image):
		image.release()
		cv2.destroyAllWindows()



	def aruco_display(self, corners, ids, rejected, image):
		
		if len(corners) > 0:
			
			ids = ids.flatten()
			
			for (markerCorner, markerID) in zip(corners, ids):
				
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners
				
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
				cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
				cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
				cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
				
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
				
				cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
					0.5, (0, 255, 0), 2)
				print("[Inference] ArUco marker ID: {}".format(markerID))
				
		return image



	def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
		parameters = cv2.aruco.DetectorParameters()
		


		corners, self.ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
			#cameraMatrix=matrix_coefficients,
			#distCoeff=distortion_coefficients
				)
	
		if len(corners) > 0:
			self.is_detected = True
			
			for i in range(0, len(self.ids)):
				#for real marker 0.105 , in gazebo 0.0325
				self.rvec, self.tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.15 , matrix_coefficients,
																		distortion_coefficients)
				
				#detect central position
				#self.estimate_central_position(rvec, tvec)
				self.tvec_array.append(self.tvec)
				self.rvec_array.append(self.rvec)
				
				cv2.aruco.drawDetectedMarkers(frame, corners) 

				cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, self.rvec, self.tvec, 0.01 ) 
				
		else:
			self.is_detected = False
				
		return frame
	

	
	def get_rotation(self):
		if(self.rvec is not None):
			return self.rvec
		

	def get_translation(self):
		if(self.tvec is not None):
			return self.tvec
		
	def get_rotation_array(self):
		if(self.rvec_array is not None):
			temp = self.rvec_array
			self.rvec_array = []
			return temp
		

	def get_translation_array(self):
		if(self.tvec_array is not None):
			temp = self.tvec_array
			self.tvec_array = []
			return temp
	

	def get_ids(self):
		if(self.ids is not None):
			return self.ids

	def flag_is_detected(self):
		return self.is_detected
	
		

	