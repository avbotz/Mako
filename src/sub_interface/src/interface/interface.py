#!/usr/bin/env python
from flask import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy as ros
import cv2
import os
import time

app = Flask(__name__)

FRONTX_RAW = 5472
FRONTY_RAW = 3648
FRONTX_GOAL = 1080
FRONTY_GOAL = 720

DOWNX_RAW = 1288
DOWNY_RAW = 964
DOWNX_GOAL = 640
DOWNY_GOAL = 480

image_converter = None


class ImageConverter:

    def __init__(self):
        self.bridge = CvBridge()
        self.front_pub = ros.Subscriber("front_camera", Image, self.set_front)
        self.down_pub = ros.Subscriber("down_camera", Image, self.set_down)
        self.front_img = None
        self.down_img = None

    def set_front(self, data):
        self.front_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        print("Generating front image.")
    
    def set_down(self, data):
        self.down_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        print("Generating down image.")

    def convert_front_stream(self):
        print("Beginning front stream.")
        while not ros.is_shutdown():
            front_img_cv = self.front_img
            front_img_cv = cv2.resize(front_img_cv, (FRONTX_GOAL, FRONTY_GOAL))
            # Don't change the following format, it is for web motion jpeg and has nothing
            # to do with input format. 
            ret, frame = cv2.imencode(".jpg", front_img_cv) 
            frame = frame.tobytes()
            # '--frame' is the boundary that marks replacing the existing content
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def convert_down_stream(self):
        print("Beginning down stream.")
        while not ros.is_shutdown():
            down_img_cv = self.down_img
            down_img_cv = cv2.resize(down_img_cv, (DOWNX_GOAL, DOWNY_GOAL))
            # Don't change the following format, it is for web motion jpeg and has nothing
            # to do with input format. 
            ret, frame = cv2.imencode(".jpg", down_img_cv) 
            frame = frame.tobytes()
            # '--frame' is the boundary that marks replacing the existing content
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


# Creates a response wrapper around the stream and routes to localhost:PORT/front_stream
# MIME type of multipart/x-mixed-replace replaces existing content with new stream data
# every time '--<boundary>' ('--frame') is displayed.
# HINT: go to localhost:PORT/front_stream to see just the front stream
@app.route("/front_stream")
def front_stream():
    global image_converter
    return Response(image_converter.convert_front_stream(), 
            mimetype='multipart/x-mixed-replace; boundary=frame')


# Creates a response wrapper around the stream and routes to localhost:PORT/down_stream
# MIME type of multipart/x-mixed-replace replaces existing content with new stream data
# every time '--<boundary>' ('--frame') is displayed.
# HINT: go to localhost:PORT/down_stream to see just the down stream
@app.route("/down_stream")
def down_stream():
    global image_converter
    return Response(image_converter.convert_down_stream(), 
            mimetype='multipart/x-mixed-replace; boundary=frame')


# index.html references both the streams.
@app.route("/")
def index():
    print("Beginning interfacing server.")
    return render_template("index.html")


def main():
    global image_converter
    ros.init_node('interface_node')
    image_converter = ImageConverter()
    app.run(host='0.0.0.0')
    ros.spin()


