#!/usr/bin/env python3

import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def _install_ultralytics() -> None:
    try:
        subprocess.check_call(["python3", "-m", "pip", "install", "--user", "ultralytics"])
        return
    except Exception:
        pass

    print("[INFO] 'pip' no está disponible. Intentando instalar python3-pip con apt...")
    subprocess.check_call(["sudo", "apt", "update"])
    subprocess.check_call(["sudo", "apt", "install", "-y", "python3-pip"])
    subprocess.check_call(["python3", "-m", "pip", "install", "--user", "ultralytics"])


try:
    from ultralytics import YOLO
except ImportError:
    try:
        print("[INFO] 'ultralytics' no está instalado. Intentando instalar automáticamente...")
        _install_ultralytics()
        print("[INFO] 'ultralytics' se ha instalado con éxito.")
        from ultralytics import YOLO
    except Exception as e:
        print(f"[ERROR] No se pudo instalar 'ultralytics' automáticamente: {e}")
        sys.exit(1)


class UnoDetectorNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        # Params
        self.package_name = rospy.get_param("~package_name", "uno_detector_ros")
        self.image_topic = rospy.get_param("~image_topic", "/uno_detector/image")
        self.conf = float(rospy.get_param("~conf", 0.25))
        self.imgsz = int(rospy.get_param("~imgsz", 640))

        self.display = bool(rospy.get_param("~display", False))
        self.print_to_terminal = bool(rospy.get_param("~print_to_terminal", False))
        self.publish_annotated = bool(rospy.get_param("~publish_annotated", True))
        self.publish_detections = bool(rospy.get_param("~publish_detections", True))

        # Weights
        self.weights_path = self._resolve_weights_path(
            rospy.get_param("~weights", "best_20260517_135309.pt")
        )

        rospy.loginfo("Loading YOLO model from %s", self.weights_path)
        self.model = YOLO(str(self.weights_path))
        rospy.loginfo("Model classes: %s", self.model.names)

        # Publishers
        self.annotated_pub = rospy.Publisher("~image_annotated", CompressedImage, queue_size=1) \
            if self.publish_annotated else None

        self.detections_pub = rospy.Publisher("~detections", String, queue_size=10) \
            if self.publish_detections else None

        # Subscriber ONLY compressed
        self.subscriber = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        rospy.loginfo("Subscribed to compressed topic: %s", self.image_topic)

        if self.display:
            cv2.namedWindow("UNO Detector", cv2.WINDOW_NORMAL)

        rospy.loginfo("UNO detector node ready.")

    def _resolve_weights_path(self, weights: str) -> Path:
        path = Path(weights)
        if path.is_absolute():
            return path

        package_root = Path(rospkg.RosPack().get_path(self.package_name))
        resolved = package_root / weights

        if not resolved.exists():
            raise FileNotFoundError(f"Weights file not found: {resolved}")

        return resolved

    def image_callback(self, msg: CompressedImage) -> None:
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            rospy.logwarn("Failed to decode compressed image")
            return

        # YOLO inference
        results = self.model.predict(
            source=frame,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False
        )

        result = results[0]
        annotated = result.plot()

        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls.item())
                label = self.model.names.get(class_id, str(class_id))
                confidence = float(box.conf.item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                detections.append({
                    "class_id": class_id,
                    "label": label,
                    "confidence": confidence,
                    "bbox_xyxy": [x1, y1, x2, y2],
                })

        # Keep only the single highest-confidence detection (publish one card at a time)
        if detections:
            best = max(detections, key=lambda d: d["confidence"])
            detections = [best]

        # Publish detections
        if self.detections_pub is not None:
            payload = {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id,
                "count": len(detections),
                "detections": detections,
            }
            self.detections_pub.publish(String(data=json.dumps(payload)))

        # Print terminal
        if self.print_to_terminal and detections:
            for d in detections:
                rospy.loginfo(
                    "Detected: %s conf=%.2f bbox=%s",
                    d["label"], d["confidence"], d["bbox_xyxy"]
                )

        # Show image (debug only)
        if self.display:
            cv2.imshow("UNO Detector", annotated)
            cv2.waitKey(1)

        # Publish annotated image (compressed)
        if self.annotated_pub is not None:
            ok, buffer = cv2.imencode(".jpg", annotated)
            if ok:
                out_msg = CompressedImage()
                out_msg.header = msg.header
                out_msg.format = "jpeg"
                out_msg.data = buffer.tobytes()
                self.annotated_pub.publish(out_msg)


def main():
    rospy.init_node("uno_detector")
    UnoDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
