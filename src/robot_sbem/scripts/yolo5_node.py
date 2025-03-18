# #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

import numpy as np
import cv2
import time



class ObjectDetection(Node):
    """
    Class implements Yolo5 model to make inferences on a youtube video using OpenCV.
    """
    
    def __init__(self):
        super().__init__("yolov5_node")
        #publisher
        self.image_pub = self.create_publisher(Image, "yolov5/image", 10)

        self.nodename = "yolov5_detection"
        self.get_logger().info(f"-I- {self.nodename} started")

        #import model
        self.model = self.load_model()
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("\n\nDevice Used:",self.device)

        #self.cap = cv2.VideoCapture(0)

        self.bridge = CvBridge()
        
        # subscriptions
        self.subscription = self.create_subscription(Image, "/camera/image_raw/uncompressed", self.listener_callback, 1)

        #part implemented by me

        



    def listener_callback(self, data):
        self.get_logger().info("Got Image")
        
        current_frame = self.bridge.imgmsg_to_cv2(data)
        # processed_image = self.model(current_frame)
        # result = self.br.cv2_to_imgmsg(processed_image.ims[0])

        #part implemented by me

        
        self.get_logger().info("detecting..")
        start_time = time.perf_counter()
        
        results = self.score_frame(current_frame)
        current_frame = self.plot_boxes(results, current_frame)
        end_time = time.perf_counter()
        fps = 1 / np.round(end_time - start_time, 3)
        cv2.putText(current_frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
        cv2.imshow("img", current_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("stop")
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(current_frame,"bgr8"))

        """"
        if self.cap.isOpened():
            self.get_logger().info("detecting..")
            start_time = time.perf_counter()
            ret, frame = self.cap.read()
            if ret:
                results = self.score_frame(frame)
                frame = self.plot_boxes(results, frame)
                end_time = time.perf_counter()
                fps = 1 / np.round(end_time - start_time, 3)
                cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
                cv2.imshow("img", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("stop")

                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
        """
        


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
        image_subscriber = ObjectDetection()
        rclpy.spin(image_subscriber)
    except rclpy.exceptions.ROSInterruptException:
        pass

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




