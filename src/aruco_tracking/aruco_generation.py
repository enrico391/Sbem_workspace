import cv2


def generate_single_marker(aruco_dict):
   marker_size = int(input("Enter the marker size: "))
   marker_id = int(input("Enter the marker ID: "))

   marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id,
    marker_size)

   cv2.imwrite("marker_{}.png".format(marker_id), marker_img)

   marker_img = cv2.imread("marker_{}.png".format(marker_id))

   cv2.imshow("Marker", marker_img)

   print("Dimensions:", marker_img.shape)

   cv2.waitKey(0)


def main():
   aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

   generate_single_marker(aruco_dict)


if __name__ == "__main__":
   main()