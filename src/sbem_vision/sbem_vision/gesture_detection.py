import cv2
import numpy as np
import mediapipe as mp
import tensorflow as tf
from tensorflow.keras.models import load_model



import rclpy
from rclpy.node import Node
import torch
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time


class GestureDetection(Node):
    """
    Class implements gesture hand detection
    """
    
    def __init__(self):
        super().__init__("gesture_node")
        #publishers
        self.image_pub = self.create_publisher(Image, "/image/gesture", 10)
        self.voice_pub = self.create_publisher(String, "output/string", 10)

        self.nodename = "gesture_node"
        self.get_logger().info(f"-I- {self.nodename} started")

        #import model
        
        # initialize mediapipe
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils

        # Load the gesture recognizer model
        self.model = load_model('src/sbem_vision/sbem_vision/mp_hand_gesture')

        # Load class names
        f = open('src/sbem_vision/sbem_vision/gesture.names', 'r')
        self.classNames = f.read().split('\n')
        f.close()
        print(self.classNames)


        #self.cap = cv2.VideoCapture(0)

        self.bridge = CvBridge()
        
        self.get_logger().info("Processing...")

        self.voice_message = String()
        self.previous_voice_message = String()

        # subscriptions
        self.subscription = self.create_subscription(Image, "/camera/image_raw/uncompressed", self.listener_callback, 1)


        



    def listener_callback(self, data):
        
        
        current_frame = self.bridge.imgmsg_to_cv2(data)
        # processed_image = self.model(current_frame)
        # result = self.br.cv2_to_imgmsg(processed_image.ims[0])

        #part implemented by me

        x, y, c = current_frame.shape

        # Flip the frame vertically
        current_frame = cv2.flip(current_frame, 1)
        framergb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

        # Get hand landmark prediction
        result = self.hands.process(framergb)

        # print(result)

        className = ''

        # post process the result
        if result.multi_hand_landmarks:
            landmarks = []
            for handslms in result.multi_hand_landmarks:
                for lm in handslms.landmark:
                    # print(id, lm)
                    lmx = int(lm.x * x)
                    lmy = int(lm.y * y)

                    landmarks.append([lmx, lmy])

                # Drawing landmarks on frames
                self.mpDraw.draw_landmarks(current_frame, handslms, self.mpHands.HAND_CONNECTIONS)

                # Predict gesture
                prediction = self.model.predict([landmarks])
                # print(prediction)
                classID = np.argmax(prediction)
                className = self.classNames[classID]
                self.get_logger().info(f"Detected : {className}" )

                #create message to send to topic that create voice 
                self.voice_message.data = className

                #use this to not send multiple same phrase to node voice
                if(self.previous_voice_message.data != self.voice_message.data):
                    self.voice_pub.publish(self.voice_message)

                self.previous_voice_message.data = self.voice_message.data
        else:
            self.previous_voice_message = String()

        # show the prediction on the frame
        cv2.putText(current_frame, className, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (0,0,255), 2, cv2.LINE_AA)
        
        # Show the final output
        #cv2.imshow("Output", current_frame) 

        if cv2.waitKey(1) == ord('q'):
            self.get_logger().info(f"Error detected in cv" )
            

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(current_frame,"bgr8"))

        


    def load_model(self):
        """
        Loads Yolo5 model from pytorch hub.
        :return: Trained Pytorch model.
        """
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        return model


    def score_frame(self, frame):
        """
        Takes a single frame as input, and scores the frame using yolo5 model.
        :param frame: input frame in numpy/list/tuple format.
        :return: Labels and Coordinates of objects detected by model in the frame.
        """
        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
     
        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord


    def class_to_label(self, x):
        """
        For a given label value, return corresponding string label.
        :param x: numeric label
        :return: corresponding string label
        """
        return self.classes[int(x)]


    def plot_boxes(self, results, frame):
        """
        Takes a frame and its results as input, and plots the bounding boxes and label on to the frame.
        :param results: contains labels and coordinates predicted by model on the given frame.
        :param frame: Frame which has been scored.
        :return: Frame with bounding boxes and labels ploted on it.
        """
        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.2:
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

        return frame



        
def main(args=None):
    rclpy.init(args=args)
    try:
        image_subscriber = GestureDetection()
        rclpy.spin(image_subscriber)
    except rclpy.exceptions.ROSInterruptException:
        pass

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

