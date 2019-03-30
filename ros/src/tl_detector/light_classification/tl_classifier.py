import rospy
import cv2
import time
import numpy as np
import tensorflow as tf
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self, is_site):
        # load classifier
        if is_site:
            graph_path = 'light_classification/trained_tf_model/ssd_sim/frozen_inference_graph.pb'
        else:
            graph_path = 'light_classification/trained_tf_model/ssd_udacity/frozen_inference_graph.pb'
        self.threshold = .5
        self.graph = tf.Graph()
        
        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_path, 'rb') as f:
                graph_def.ParseFromString(f.read())
                tf.import_graph_def(graph_def, name='tl_classifier')
                
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')
            
        self.sess = tf.Session(graph=self.graph)
        self.light_code_dict = {1: TrafficLight.GREEN, 2: TrafficLight.RED, 3: TrafficLight.YELLOW, 4: TrafficLight.UNKNOWN}
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = time.time()
            [boxes, scores, classes, num_detections] = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand}
            )
            end = time.time()
            c = end - start
            rospy.loginfo('took {} seconds to classify traffic light'.format(c))
            
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        
        if scores[0] > self.threshold:
            return self.light_code_dict.get(classes[0], TrafficLight.UNKNOWN)
            
        return TrafficLight.UNKNOWN
