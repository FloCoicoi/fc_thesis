#!/usr/bin/env python3

"""
The original code is Luxonis's RGB Depth Alignment code example.

I modified it to publish the images on ROS so they can be recorded as rosbags.
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import depthai as dai

# Optional. If set (True), the ColorCamera is downscaled from 1080p to 720p.
# Otherwise (False), the aligned depth is automatically upscaled to 1080p
downscaleColor = True
fps = 30
# The disparity is computed at this resolution, then upscaled to RGB resolution
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_480_P

# Init ros node and publishers
rospy.init_node("luxonis")
imgPublishers = {}
imgPublishers["rgb"] = rospy.Publisher("~rgb/image", Image, queue_size=5)
imgPublishers["depth"] = rospy.Publisher("~depth/centered/image", Image, queue_size=5)
imgPublishers["left"] = rospy.Publisher("~stereo/left/image", Image, queue_size=5)
imgPublishers["right"] = rospy.Publisher("~stereo/right/image", Image, queue_size=5)
imgPublishers["aligned"] = rospy.Publisher("~depth/aligned_to_rgb", Image, queue_size=5)

camInfoPublishers = {}
camInfoPublishers["rgb"] = rospy.Publisher("~rgb/cam_info", CameraInfo, queue_size=1)
camInfoPublishers["left"] = rospy.Publisher("~stereo/left/cam_info", CameraInfo, queue_size=1)
camInfoPublishers["depth"] = rospy.Publisher("~depth/centered/cam_info", CameraInfo, queue_size=1)
bridge = CvBridge()

# img messages: encodings and frames
imgEncoding = {}
imgEncoding["rgb"] = "rgb8"
imgEncoding["aligned"] = "16UC1"
imgEncoding["right"] = "8UC1"
imgEncoding["left"] = "8UC1"
imgEncoding["depth"] = "16UC1"
imgFrameId = {}
imgFrameId["rgb"] = "/luxonis_camera/rgb"
imgFrameId["aligned"] = "/luxonis_camera/rgb"
imgFrameId["right"] = "/luxonis_camera/right"
imgFrameId["left"] = "/luxonis_camera/left"
imgFrameId["depth"] = "/luxonis_camera/rgb"

# Create pipeline
pipeline = dai.Pipeline()
device = dai.Device()
queueNames = []

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
alignedDepth = pipeline.create(dai.node.StereoDepth)
stereoDepth = pipeline.create(dai.node.StereoDepth)

rgbOut = pipeline.create(dai.node.XLinkOut)
leftOut = pipeline.create(dai.node.XLinkOut)
rightOut = pipeline.create(dai.node.XLinkOut)
depthOut = pipeline.create(dai.node.XLinkOut)
alignedOut = pipeline.create(dai.node.XLinkOut)
rgbOut.setStreamName("rgb")
leftOut.setStreamName("left")
rightOut.setStreamName("right")
depthOut.setStreamName("depth")
alignedOut.setStreamName("aligned")

# Set input properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setFps(fps)
if downscaleColor:
    camRgb.setIspScale(2, 3)
# For now, RGB needs fixed focus to properly align with depth.
# This value was used during calibration
try:
    calibData = device.readCalibration2()
    lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.RGB)
    if lensPosition:
        camRgb.initialControl.setManualFocus(lensPosition)
except:
    raise
left.setResolution(monoResolution)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
left.setFps(fps)
right.setResolution(monoResolution)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
right.setFps(fps)

# Depth computation params
alignedDepth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
alignedDepth.setLeftRightCheck(True)  # LR-check is required for depth alignment
alignedDepth.setSubpixel(True)
alignedDepth.setDepthAlign(dai.CameraBoardSocket.RGB)
stereoDepth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# LR-check is required for depth alignment
stereoDepth.setLeftRightCheck(True)
stereoDepth.setSubpixel(True)

# Linking
camRgb.isp.link(rgbOut.input)
left.out.link(stereoDepth.left)
right.out.link(stereoDepth.right)
left.out.link(alignedDepth.left)
right.out.link(alignedDepth.right)
stereoDepth.syncedLeft.link(leftOut.input)
stereoDepth.syncedRight.link(rightOut.input)
# alignedDepth.syncedLeft.link(leftOut.input)
# alignedDepth.syncedRight.link(rightOut.input)
stereoDepth.depth.link(depthOut.input)
alignedDepth.depth.link(alignedOut.input)


def publish_img_msg(bridge, publisher, msg, imId, stamp=rospy.Time.now(), desired_encoding=None):
    """Convert image and publish ROS message.

    Parameters
    ----------
    publisher: Publisher
        ROS publisher
    msg: numpy.array
        image to publish
    imId: str
        frame id
    desired_encoding: str
        encoding
    """
    if desired_encoding is not None:
        # msg = ros_numpy.image.numpy_to_image(msg, desired_encoding)
        msg = bridge.cv2_to_imgmsg(msg, desired_encoding)
    msg.header.frame_id = imId
    msg.header.stamp = stamp
    publisher.publish(msg)


# CameraInfo messages
camInfos = {}
rgbcamInfo = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
camInfos["rgb"] = [item for sublist in rgbcamInfo for item in sublist]
leftcamInfo = calibData.getCameraIntrinsics(dai.CameraBoardSocket.LEFT)
camInfos["left"] = [item for sublist in leftcamInfo for item in sublist]
depthcamInfo = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT)
camInfos["depth"] = [item for sublist in depthcamInfo for item in sublist]

# Connect to device and start pipeline
with device:
    device.startPipeline(pipeline)

    ## Can use a .getTimestamp() to get the end of exposure time
    while not rospy.is_shutdown():

        # Get measurements and publish them
        queueEvents = device.getQueueEvents(("rgb", "left", "right", "depth", "aligned"))  # "disp"
        for imgName in queueEvents:
            packets = device.getOutputQueue(imgName).tryGetAll()
            for measurement in packets:
                curFrame = measurement.getCvFrame()
                timestamp = rospy.Time(measurement.getTimestamp().total_seconds())
                publish_img_msg(
                    bridge, imgPublishers[imgName], curFrame, imgFrameId[imgName], timestamp, imgEncoding[imgName]
                )
                if imgName in camInfoPublishers.keys():
                    # if imgName == "depth":
                    #     rospy.loginfo(
                    #         "depth width, height = "
                    #         + str(measurement.getWidth())
                    #         + ", "
                    #         + str(measurement.getHeight())
                    #     )
                    #     continue
                    new_info = CameraInfo()
                    new_info.width = measurement.getWidth()
                    new_info.height = measurement.getHeight()
                    new_info.K = camInfos[imgName]
                    new_info.header.frame_id = imgFrameId[imgName]
                    new_info.header.stamp = timestamp
                    camInfoPublishers[imgName].publish(new_info)

        if cv2.waitKey(1) == ord("q"):
            break
