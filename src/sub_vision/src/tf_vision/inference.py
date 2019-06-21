#!/usr/bin/env python
import rospy as ros
import geometry_msgs.msg as geo 
import sensor_msgs.msg as sen 

import numpy as np
import sys
import tensorflow as tf
import cv2

from PIL import Image

import map_util
import vis_util

PATH_TO_CKPT = 'models/vis1.pb'
PATH_TO_LABELS = 'models/vis1.pbtxt'
NUM_CLASSES = 4


class Inference: 

    def __init__(self):
        self.subscriber = ros.Subscriber("front_camera",
                sen.Image, self.capture_image, queue_size=1)

    def capture_image(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.image_np = cv2.imdecode(np_arr, 1)
        print(self.image_np)

    def load_image(self, image):
        (im_width, im_height) = image.size
        return np.array(image).reshape(
                (im_height, im_width, 3)).astype(np.uint8)

    def setup(self):
        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        label_map = map_util.load_labelmap(PATH_TO_LABELS)
        categories = map_util.convert_label_map_to_categories(label_map, 
                max_num_classes=NUM_CLASSES, use_display_name=True)
        category_index = map_util.create_category_index(categories)

    def detect(self):
        self.setup()

        with self.detection_graph.as_default():
            config = tf.ConfigProto()
            config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

            with tf.Session(graph=self.detection_graph, config=config) as sess:
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                while (True):
                    ros.spin()
                    image = self.image_np
                    image_np = self.load_image(image)
                    image_np_expanded = np.expand_dims(image_np, axis=0)
                    (boxes, scores, classes, num) = sess.run(
                            [detection_boxes, detection_scores, detection_classes, num_detections],
                            feed_dict={image_tensor: image_np_expanded})
                    width, height = image.size
                    newbox = np.squeeze(boxes)
                    finalboxes = newbox[~(newbox==0).all(1)]
                    numboxes = len(finalboxes)
                    for x in range(0,numboxes):
                        ymin = boxes[0][x][0]*height
                        xmin = boxes[0][x][1]*width
                        ymax = boxes[0][x][2]*height
                        xmax = boxes[0][x][3]*width

def main():
    pub = ros.Publisher('tf_perception', geo.Point, queue_size=10)
    ros.init_node('vision_inference_node')
    inf = Inference()
    inf.detect()

