

from cv_bridge              import CvBridge, CvBridgeError
import cv2



def main():
        cap = cv2.VideoCapture(0)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        num = 0

        while cap.isOpened():
            ret, img = cap.read()
            h, w, _ = img.shape
            width = 1000
            height = int(width*(h/w))
            
            img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
            
            k = cv2.waitKey(5)

            if k == ord('s'): # wait for 's' key to save and exit
                cv2.imwrite('/home/morolinux/Documents/sbem_project_ws/src/aruco_tracking/aruco_calib_realCamera/images/' + str(num) + '.png', img)
                print("image saved!")
                num += 1
            cv2.imshow('Img',img)

        
    


main()