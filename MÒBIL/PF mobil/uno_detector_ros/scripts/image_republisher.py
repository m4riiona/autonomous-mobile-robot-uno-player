#!/usr/bin/env python3

from typing import Optional

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class ImageRepublisher:
    """Subscribe to a camera image topic (compressed) and republish to a standard topic.

    This allows decoupling the camera source from processing nodes.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.input_topic = rospy.get_param("~image_topic", "/camera/image_raw/compressed")
        self.input_format = rospy.get_param("~input_format", "auto").strip().lower()
        self.output_topic = rospy.get_param("~output_topic", "/uno_detector/image")
        self.queue_size = int(rospy.get_param("~queue_size", 1))

        if self.input_format == "auto":
            self.input_format = "compressed" if self.input_topic.endswith("/compressed") else "raw"

        self.pub = rospy.Publisher(self.output_topic, CompressedImage, queue_size=self.queue_size)
        if self.input_format == "raw":
            self.sub = rospy.Subscriber(self.input_topic, Image, self.cb_raw, queue_size=self.queue_size)
        else:
            self.sub = rospy.Subscriber(self.input_topic, CompressedImage, self.cb_compressed, queue_size=self.queue_size)

        rospy.loginfo(
            "ImageRepublisher: subscribing %s (%s) -> publishing %s",
            self.input_topic,
            self.input_format,
            self.output_topic,
        )

    def cb_compressed(self, msg: CompressedImage) -> None:
        # Simple passthrough, preserve header and data
        out = CompressedImage()
        out.header = msg.header
        out.format = msg.format
        out.data = msg.data
        self.pub.publish(out)

    def cb_raw(self, msg: Image) -> None:
        compressed = self.bridge.cv2_to_compressed_imgmsg(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))
        compressed.header = msg.header
        self.pub.publish(compressed)


def main():
    rospy.init_node("image_republisher")
    ImageRepublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
