#!/usr/bin/env python
from flask import *
from flask_socketio import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy as ros
import base64
import eventlet
import cv2
import os
import time

eventlet.sleep(0)
eventlet.monkey_patch()

app = Flask(__name__)
socket = SocketIO(app)

FRONTX_RAW = 5472
FRONTY_RAW = 3648
FRONTX_GOAL = 1080
FRONTY_GOAL = 720

DOWNX_RAW = 1288
DOWNY_RAW = 964
DOWNX_GOAL = 640
DOWNY_GOAL = 480

image_converter = None
bridge = None


class FrontImageConverter:

    def __init__(self):
        self.front_pub = ros.Subscriber("front_camera", Image, self.set_front)

    def set_front(self, data):
        # global socket
        global bridge
        front_img = bridge.imgmsg_to_cv2(data, "bgr8")
        front_img_cv = cv2.resize(front_img, (FRONTX_GOAL, FRONTY_GOAL))
        _, buf = cv2.imencode(".jpg", front_img_cv, 
                [int(cv2.IMWRITE_JPEG_QUALITY), 90])  
        buf_encoded = base64.b64encode(buf)
        socket.emit('front_img', buf_encoded, broadcast=True)
        socket.sleep(0)
        print("Generating front image.")

class DownImageConverter:
    
    def __init__(self):
        self.down_pub = ros.Subscriber("down_camera", Image, self.set_down)

    def set_down(self, data):
        # global socket
        global bridge
        down_img = bridge.imgmsg_to_cv2(data, "bgr8")
        down_img_cv = cv2.resize(down_img, (DOWNX_GOAL, DOWNY_GOAL))
        _, buf = cv2.imencode(".jpg", down_img_cv, 
                [int(cv2.IMWRITE_JPEG_QUALITY), 90])  
        buf_encoded = base64.b64encode(buf)
        socket.emit('down_img', buf_encoded, broadcast=True)
        socket.sleep(0)
        print("Generating down image.")


# index.html references both the streams.
@app.route("/")
def index():
    print("Beginning interfacing server.")
    return render_template("index.html")


def main():
    global bridge
    ros.init_node('interface_node')
    bridge = CvBridge()
    image_converter = FrontImageConverter()
    image_converter1 = DownImageConverter()
    # app.run(host='0.0.0.0')
    socket.run(app, host='0.0.0.0', port=5000, debug=True)
    ros.spin()
