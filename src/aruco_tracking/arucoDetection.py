import numpy as np
import time
import cv2
import math

from arucoPoseEstimation import ArucoPoseEstimation


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

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def aruco_display(corners, ids, rejected, image):
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

			trans = processImage.get_translation()


			cv2.putText(image, f"{trans[0][0][2]}",(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.6, (0, 255, 0), 2)
			print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return image




aruco_type = "DICT_6X6_250" # FOR SIMULATION "DICT_6X6_250"

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

arucoParams = cv2.aruco.DetectorParameters()



# VALUE FOR LOGITECH WEBCAM
intrinsic_camera = np.array(((1.06375410e+03 , 0.0 , 4.50310306e+02),(0.0 , 1.07067102e+03 , 2.96850256e+02),(0.0 , 0.0 , 1.0)))
distortion  = np.array((-3.14273296e-02 , -2.31123892e-01  , 4.64970586e-03  , 1.89295234e-04 , 1.81898326e+00))

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

processImage = ArucoPoseEstimation()


while cap.isOpened():
    
	ret, img = cap.read()

	h, w, _ = img.shape

	width = 1000
	height = int(width*(h/w))
	img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

	processImage.pose_estimation(img,ARUCO_DICT[aruco_type],intrinsic_camera,distortion)
 
	corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

	object_points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
	
	normalize_corners = []

	if len(corners) > 0:
			for i in range(0, len(ids)):
				normalize_corners.append(np.array(corners[i].flatten()))

				#for real marker 0.105 , in gazebo 0.0325
				rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.15 , intrinsic_camera,
																		distortion)
				



				#print(np.rad2deg(quaternion_from_euler(rvec.flatten()[0], rvec.flatten()[1], rvec.flatten()[2])))
				#print(f"{np.deg2rad(rvec[0][0])}")
				#print(f"{tvec.flatten()}")




				 
				# Get rotation and translation vectors of the marker
				rvec_marker = rvec
				tvec_marker = tvec
	
				# Invert the transformation (to get camera position relative to the marker)
				rvec_camera, _ = cv2.Rodrigues(rvec_marker)
				tvec_camera = -np.dot(np.squeeze(rvec_camera.T), np.squeeze(tvec_marker.T))
				

				print("Camera Orientation (relative to marker):", np.rad2deg(rvec_camera))
				# Print camera position relative to the marker
				print("Camera Position (relative to marker):", tvec_camera)
				
	

	detected_markers = aruco_display(corners, ids, rejected, img)

	cv2.imshow("Image", detected_markers)

	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
	    break

cv2.destroyAllWindows()
cap.release()



def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


