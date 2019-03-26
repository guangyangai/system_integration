import rospy
import cv2
import time
import numpy as np
import tensorflow as tf
from keras import backend
from keras import layers
from keras.models import load_model
from keras.applications.vgg16 import preprocess_input
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self, is_site):
        #TODO load classifier
        if is_site:
            self.model = load_model('../../../data_science/models/site_model.h5')
        else:
            self.model = load_model('../../../data_science/models/simulator_model.h5')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # We know the location of the light 
        return TrafficLight.UNKNOWN
