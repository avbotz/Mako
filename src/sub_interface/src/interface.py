#!/usr/bin/env python
from flask import *
import cv2
import glob
import os
import time

app = Flask(__name__)

# set this to the path (including glob) of front camera images
FRONT_GLOB = "/*.jpg"
# set this to the resolution of the front camera images
FRONTX_RAW = 1920
FRONTY_RAW = 1080
# tune these to your liking, but beware that currently more than 2x480p images or 1x720p image is laggy
FRONTX_GOAL = 1280
FRONTY_GOAL = 720
FRONTX_SCALE = FRONTX_GOAL/FRONTX_RAW
FRONTY_SCALE = FRONTY_GOAL/FRONTY_RAW

# set this to the path (including glob) of front camera images
FRONT_GLOB = "/home/kalyan/mako/src/sub_interface/src/*.jpg"
# set this to the resolution of the down camera images
DOWNX_RAW = 1920
DOWNY_RAW = 1080
# tune these to your liking, but beware that currently more than 2x480p images or 1x720p image is laggy
DOWNX_GOAL = 1280
DOWNY_GOAL = 720
DOWNX_SCALE = DOWNX_GOAL/DOWNX_RAW
DOWNY_SCALE = DOWNY_GOAL/DOWNY_RAW


# generates a flask stream of content with yield, special MIME type used
# to replace existing content with each new frame. See front_stream()
def generate_front_stream():
    while True:
        list_of_files = glob.glob(FRONT_GLOB)
        latest_file = max(list_of_files, key=os.path.getctime)
        img = cv2.imread(latest_file, 1)
        img = cv2.resize(img, None, fx=FRONTX_SCALE, fy = FRONTY_SCALE)
        # don't change the following format, it is for web motion jpeg and has nothing
        # to do with input format. For changing file format to read images, change the
        # FRONT_GLOB and DOWN_GLOB global vars
        ret, frame = cv2.imencode(".jpg", img) 
        frame = frame.tobytes()
        # '--frame' is the boundary that marks replacing the existing content
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


# generates a flask stream of content with yield, special MIME type used
# to replace existing content with each new frame. See front_stream()
def generate_down_stream():
    while True:
        list_of_files = glob.glob(DOWN_GLOB)
        latest_file = max(list_of_files, key=os.path.getctime)
        img = cv2.imread(latest_file, 1)
        img = cv2.resize(img, None, fx=DOWNX_SCALE, fy = DOWNY_SCALE)
        # don't change the following format, it is for web motion jpeg and has nothing
        # to do with input format. For changing file format to read images, change the
        # FRONT_GLOB and DOWN_GLOB global vars
        ret, frame = cv2.imencode(".jpg", img) 
        frame = frame.tobytes()
        # '--frame' is the boundary that marks replacing the existing content
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


# creates a response wrapper around the stream and routes to localhost:PORT/front_stream
# MIME type of multipart/x-mixed-replace replaces existing content with new stream data
# every time '--<boundary>' ('--frame') is displayed
# HINT: go to localhost:PORT/front_stream to see just the front stream
@app.route("/front_stream")
def front_stream():
    return Response(generate_front_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')


# creates a response wrapper around the stream and routes to localhost:PORT/down_stream
# MIME type of multipart/x-mixed-replace replaces existing content with new stream data
# every time '--<boundary>' ('--frame') is displayed
@app.route("/down_stream")
# HINT: go to localhost:PORT/down_stream to see just the down stream
def down_stream():
    return Response(generate_down_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')


# index.html references both the streams
@app.route("/")
def main():
    return render_template("index.html")

if __name__ == "__main__":
    app.run()

