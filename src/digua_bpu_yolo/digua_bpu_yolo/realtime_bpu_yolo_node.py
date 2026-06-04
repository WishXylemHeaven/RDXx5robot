#!/usr/bin/env python3
import json
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


try:
    from hobot_dnn import pyeasy_dnn as dnn
except Exception:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn


def sigmoid(x):
    x = np.clip(x, -50, 50)
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x, axis=-1):
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / np.sum(e, axis=axis, keepdims=True)


def bgr2nv12_opencv(bgr_img):
    h, w = bgr_img.shape[:2]
    yuv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420).reshape(-1)

    y_size = h * w
    uv_size = y_size // 4

    y = yuv[:y_size]
    u = yuv[y_size:y_size + uv_size].reshape(h // 2, w // 2)
    v = yuv[y_size + uv_size:y_size + uv_size * 2].reshape(h // 2, w // 2)

    uv = np.empty((h // 2, w), dtype=np.uint8)
    uv[:, 0::2] = u
    uv[:, 1::2] = v

    nv12 = np.concatenate([y, uv.reshape(-1)])
    return np.ascontiguousarray(nv12)


def load_classes(path):
    names = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if ":" in line:
                line = line.split(":", 1)[1].strip()
            names.append(line)
    return names


def normalize_cls_scores(cls, mode):
    if mode == "sigmoid":
        return sigmoid(cls)
    if mode == "raw":
        return cls

    mn = float(np.min(cls))
    mx = float(np.max(cls))
    if mn < 0.0 or mx > 1.0:
        return sigmoid(cls)
    return cls


def decode_one_level(cls, reg, stride, score_threshold, cls_mode):
    h, w, c = cls.shape

    cls_scores = normalize_cls_scores(cls, cls_mode)
    class_ids = np.argmax(cls_scores, axis=-1)
    scores = np.max(cls_scores, axis=-1)

    mask = scores >= score_threshold
    if not np.any(mask):
        return (
            np.zeros((0, 4), dtype=np.float32),
            np.zeros((0,), dtype=np.float32),
            np.zeros((0,), dtype=np.int32),
        )

    ys, xs = np.where(mask)
    scores_keep = scores[ys, xs].astype(np.float32)
    class_keep = class_ids[ys, xs].astype(np.int32)

    reg_keep = reg[ys, xs, :].reshape(-1, 4, 16)
    prob = softmax(reg_keep, axis=-1)
    bins = np.arange(16, dtype=np.float32)
    dist = np.sum(prob * bins, axis=-1)

    anchor_x = (xs.astype(np.float32) + 0.5) * stride
    anchor_y = (ys.astype(np.float32) + 0.5) * stride

    left = dist[:, 0] * stride
    top = dist[:, 1] * stride
    right = dist[:, 2] * stride
    bottom = dist[:, 3] * stride

    x1 = anchor_x - left
    y1 = anchor_y - top
    x2 = anchor_x + right
    y2 = anchor_y + bottom

    boxes = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)
    boxes[:, 0::2] = np.clip(boxes[:, 0::2], 0, 640)
    boxes[:, 1::2] = np.clip(boxes[:, 1::2], 0, 640)

    return boxes, scores_keep, class_keep


def iou_one_to_many(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])

    inter_w = np.maximum(0.0, x2 - x1)
    inter_h = np.maximum(0.0, y2 - y1)
    inter = inter_w * inter_h

    area1 = max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])
    area2 = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0.0, boxes[:, 3] - boxes[:, 1])

    return inter / (area1 + area2 - inter + 1e-6)


def nms_class_aware(boxes, scores, class_ids, iou_threshold, top_k):
    keep_all = []

    for cls_id in np.unique(class_ids):
        idxs = np.where(class_ids == cls_id)[0]
        idxs = idxs[np.argsort(scores[idxs])[::-1]]

        while idxs.size > 0:
            current = idxs[0]
            keep_all.append(current)

            if len(keep_all) >= top_k:
                break

            if idxs.size == 1:
                break

            ious = iou_one_to_many(boxes[current], boxes[idxs[1:]])
            idxs = idxs[1:][ious < iou_threshold]

        if len(keep_all) >= top_k:
            break

    keep_all = np.array(keep_all, dtype=np.int32)
    if keep_all.size == 0:
        return keep_all

    keep_all = keep_all[np.argsort(scores[keep_all])[::-1]]
    return keep_all[:top_k]


class RealtimeBpuYoloNode(Node):
    def __init__(self):
        super().__init__("realtime_bpu_yolo_node")

        self.declare_parameter(
            "model_file",
            "/home/sunrise/digua_ws/models/bpu_yolov8s_oiv7/yolov8s-oiv7_bayese_640x640_nv12.bin",
        )
        self.declare_parameter(
            "classes_file",
            "/home/sunrise/digua_ws/src/digua_bpu_yolo/config/oiv7_classes.list",
        )
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/semantic/detections_json")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        self.declare_parameter("model_width", 640)
        self.declare_parameter("model_height", 640)

        self.declare_parameter("score_threshold", 0.25)
        self.declare_parameter("nms_threshold", 0.7)
        self.declare_parameter("top_k", 50)
        self.declare_parameter("cls_mode", "auto")

        self.declare_parameter("infer_fps", 3.0)
        self.declare_parameter("publish_empty", True)

        # 逗号分隔，例如 "bottle,chair,cup,footwear"；空字符串表示不过滤
        self.declare_parameter("publish_labels", "")

        self.model_file = self.get_parameter("model_file").value
        self.classes_file = self.get_parameter("classes_file").value
        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.model_width = int(self.get_parameter("model_width").value)
        self.model_height = int(self.get_parameter("model_height").value)

        self.score_threshold = float(self.get_parameter("score_threshold").value)
        self.nms_threshold = float(self.get_parameter("nms_threshold").value)
        self.top_k = int(self.get_parameter("top_k").value)
        self.cls_mode = str(self.get_parameter("cls_mode").value)

        self.infer_fps = float(self.get_parameter("infer_fps").value)
        self.publish_empty = bool(self.get_parameter("publish_empty").value)

        publish_labels_str = str(self.get_parameter("publish_labels").value).strip()
        if publish_labels_str:
            self.publish_labels = set([x.strip().lower() for x in publish_labels_str.split(",") if x.strip()])
        else:
            self.publish_labels = set()

        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_image_time = None
        self.is_busy = False

        self.classes = load_classes(self.classes_file)

        self.get_logger().info("loading BPU model...")
        self.models = dnn.load(self.model_file)
        self.model = self.models[0]
        self.get_logger().info(f"loaded model: {self.model_file}")
        self.get_logger().info(f"classes: {len(self.classes)} from {self.classes_file}")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"detections_topic: {self.detections_topic}")
        self.get_logger().info(f"score_threshold: {self.score_threshold}")
        self.get_logger().info(f"nms_threshold: {self.nms_threshold}")
        self.get_logger().info(f"top_k: {self.top_k}")
        self.get_logger().info(f"infer_fps: {self.infer_fps}")
        self.get_logger().info(f"publish_labels filter: {sorted(list(self.publish_labels))}")

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            1,
        )

        self.pub = self.create_publisher(String, self.detections_topic, 10)

        timer_period = 1.0 / max(self.infer_fps, 0.1)
        self.timer = self.create_timer(timer_period, self.infer_latest)

        self.last_stat_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg):
        self.latest_image_msg = msg
        self.latest_image_time = time.time()

    def infer_latest(self):
        if self.is_busy:
            return

        if self.latest_image_msg is None:
            return

        self.is_busy = True
        try:
            msg = self.latest_image_msg
            self.run_one_frame(msg)
        except Exception as e:
            self.get_logger().error(f"infer_latest failed: {repr(e)}")
        finally:
            self.is_busy = False

    def run_one_frame(self, image_msg):
        t0 = time.time()

        bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        orig_h, orig_w = bgr.shape[:2]

        resized = cv2.resize(
            bgr,
            (self.model_width, self.model_height),
            interpolation=cv2.INTER_AREA,
        )

        nv12 = bgr2nv12_opencv(resized)

        t1 = time.time()
        outputs = self.model.forward(nv12)
        t2 = time.time()

        detections = self.postprocess_outputs(outputs, orig_w=orig_w, orig_h=orig_h)
        t3 = time.time()

        if detections or self.publish_empty:
            payload = {
                "stamp": self.get_clock().now().nanoseconds / 1e9,
                "frame_id": self.frame_id,
                "image_width": int(orig_w),
                "image_height": int(orig_h),
                "source": "realtime_bpu_yolo",
                "detections": detections,
            }

            out_msg = String()
            out_msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub.publish(out_msg)

        self.frame_count += 1
        now = time.time()
        if now - self.last_stat_time > 2.0:
            self.get_logger().info(
                f"realtime detections={len(detections)}, "
                f"preprocess={(t1-t0)*1000:.1f}ms, "
                f"infer={(t2-t1)*1000:.1f}ms, "
                f"post={(t3-t2)*1000:.1f}ms"
            )
            for det in detections[:5]:
                self.get_logger().info(
                    f"det {det['label']} {det['confidence']:.3f} bbox={det['bbox']}"
                )
            self.last_stat_time = now

    def tensor_to_numpy(self, tensor, shape):
        arr = np.array(tensor.buffer, dtype=np.float32)
        expected = int(np.prod(shape))
        if arr.size != expected:
            arr = arr.reshape(-1)
            if arr.size != expected:
                raise RuntimeError(f"bad output size, got {arr.size}, expected {expected}, shape={shape}")
        return arr.reshape(shape)

    def postprocess_outputs(self, outputs, orig_w, orig_h):
        specs = [
            (0, 1, 80, 80, 8),
            (2, 3, 40, 40, 16),
            (4, 5, 20, 20, 32),
        ]

        all_boxes = []
        all_scores = []
        all_class_ids = []

        for cls_idx, reg_idx, h, w, stride in specs:
            cls = self.tensor_to_numpy(outputs[cls_idx], (h, w, 601))
            reg = self.tensor_to_numpy(outputs[reg_idx], (h, w, 64))

            boxes, scores, class_ids = decode_one_level(
                cls=cls,
                reg=reg,
                stride=stride,
                score_threshold=self.score_threshold,
                cls_mode=self.cls_mode,
            )

            all_boxes.append(boxes)
            all_scores.append(scores)
            all_class_ids.append(class_ids)

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        class_ids = np.concatenate(all_class_ids, axis=0)

        if len(scores) == 0:
            return []

        keep = nms_class_aware(
            boxes=boxes,
            scores=scores,
            class_ids=class_ids,
            iou_threshold=self.nms_threshold,
            top_k=self.top_k,
        )

        sx = float(orig_w) / float(self.model_width)
        sy = float(orig_h) / float(self.model_height)

        detections = []
        for i in keep:
            cls_id = int(class_ids[i])
            label = self.classes[cls_id] if 0 <= cls_id < len(self.classes) else f"class_{cls_id}"
            label_norm = label.strip().lower()

            if self.publish_labels and label_norm not in self.publish_labels:
                continue

            x1, y1, x2, y2 = [float(v) for v in boxes[i]]

            x1 *= sx
            x2 *= sx
            y1 *= sy
            y2 *= sy

            x1 = max(0.0, min(float(orig_w - 1), x1))
            x2 = max(0.0, min(float(orig_w - 1), x2))
            y1 = max(0.0, min(float(orig_h - 1), y1))
            y2 = max(0.0, min(float(orig_h - 1), y2))

            bw = max(1.0, x2 - x1)
            bh = max(1.0, y2 - y1)
            cx = x1 + bw / 2.0
            cy = y1 + bh / 2.0

            detections.append({
                "label": label_norm,
                "confidence": float(scores[i]),
                "bbox": {
                    "cx": float(cx),
                    "cy": float(cy),
                    "w": float(bw),
                    "h": float(bh),
                },
                "class_id": cls_id,
            })

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeBpuYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
