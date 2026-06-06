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




def load_aliases(path):
    """
    读取 OIV7 alias 文件。
    key 是最终发布到语义地图的名字，例如 tv；
    value 是 OIV7 类别表里的真实类别名，例如 television。
    """
    if not path:
        return {}

    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except Exception:
        return {}

    aliases = {}
    for k, values in raw.items():
        key = str(k).strip().lower()
        if not key:
            continue

        if isinstance(values, str):
            values = [values]

        out = []
        for v in values:
            v = str(v).strip().lower()
            if v:
                out.append(v)

        aliases[key] = out

    return aliases


def load_semantic_whitelist(path):
    """
    轻量解析 semantic_mapping.yaml 里的 semantic_observer_node.class_whitelist。
    不依赖 PyYAML，避免额外安装包。
    """
    labels = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except Exception:
        return labels

    in_observer = False
    in_whitelist = False

    for raw in lines:
        line = raw.rstrip("\n")
        stripped = line.strip()

        if not stripped or stripped.startswith("#"):
            continue

        if stripped.startswith("semantic_observer_node:"):
            in_observer = True
            in_whitelist = False
            continue

        # 遇到下一个顶层节点，退出 observer 区域
        if in_observer and not line.startswith(" ") and stripped.endswith(":"):
            if not stripped.startswith("semantic_observer_node:"):
                in_observer = False
                in_whitelist = False
            continue

        if not in_observer:
            continue

        if stripped.startswith("class_whitelist:"):
            in_whitelist = True
            continue

        if in_whitelist:
            if stripped.startswith("- "):
                label = stripped[2:].strip().strip('"').strip("'").lower()
                if label:
                    labels.append(label)
            elif stripped.endswith(":"):
                break

    return labels


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


def decode_one_level(cls, reg, stride, score_threshold, cls_mode, target_class_ids=None):
    """
    cls: H,W,601
    reg: H,W,64

    优化点：
    - target_class_ids 为空：仍然处理全部 601 类
    - target_class_ids 非空：只取白名单类别对应的通道做 sigmoid/argmax
    """
    h, w, c = cls.shape

    if target_class_ids:
        target_ids = np.array(target_class_ids, dtype=np.int32)
        cls_selected = cls[:, :, target_ids]          # H,W,K
        cls_scores = normalize_cls_scores(cls_selected, cls_mode)

        local_ids = np.argmax(cls_scores, axis=-1)    # H,W
        scores = np.max(cls_scores, axis=-1)          # H,W
        class_ids = target_ids[local_ids]             # H,W，映射回 601 类原始 class_id
    else:
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
        self.declare_parameter("use_semantic_whitelist", True)
        self.declare_parameter(
            "semantic_whitelist_yaml",
            "/home/sunrise/digua_ws/src/digua_semantic_mapping/config/semantic_mapping.yaml",
        )
        self.declare_parameter(
            "aliases_file",
            "/home/sunrise/digua_ws/src/digua_bpu_yolo/config/oiv7_aliases.json",
        )

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
        self.use_semantic_whitelist = bool(self.get_parameter("use_semantic_whitelist").value)
        self.semantic_whitelist_yaml = str(self.get_parameter("semantic_whitelist_yaml").value)
        self.aliases_file = str(self.get_parameter("aliases_file").value)
        self.aliases = load_aliases(self.aliases_file)

        if publish_labels_str:
            # launch 里显式传 publish_labels 时，优先用这个
            self.publish_labels = set([x.strip().lower() for x in publish_labels_str.split(",") if x.strip()])
            self.publish_labels_source = "launch publish_labels"
        else:
            self.publish_labels = set()
            self.publish_labels_source = "none"

            # 没显式传 publish_labels 时，自动读取语义地图白名单
            if self.use_semantic_whitelist:
                labels = load_semantic_whitelist(self.semantic_whitelist_yaml)
                self.publish_labels = set(labels)
                self.publish_labels_source = f"semantic whitelist: {self.semantic_whitelist_yaml}"

        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_image_time = None
        self.is_busy = False

        self.classes = load_classes(self.classes_file)

        # 把白名单类别名转换成 OIV7 class_id，用于前置过滤。
        # 同时保存 class_id -> 发布名。
        # 例如白名单 tv -> alias television -> OIV7 class_id -> 最终发布 label=tv。
        self.target_class_ids = []
        self.class_id_to_publish_label = {}

        class_name_to_ids = {}
        for idx, name in enumerate(self.classes):
            key = name.strip().lower()
            class_name_to_ids.setdefault(key, []).append(idx)

        unmatched_labels = []
        for publish_label in sorted(self.publish_labels):
            candidates = [publish_label]
            candidates.extend(self.aliases.get(publish_label, []))

            matched = False
            for cand in candidates:
                cand = cand.strip().lower()
                ids = class_name_to_ids.get(cand)
                if not ids:
                    continue

                matched = True
                for cid in ids:
                    self.target_class_ids.append(cid)
                    self.class_id_to_publish_label[cid] = publish_label

            if not matched:
                unmatched_labels.append(publish_label)

        self.target_class_ids = sorted(set(self.target_class_ids))
        self.unmatched_labels = unmatched_labels

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
        self.get_logger().info(f"publish_labels source: {self.publish_labels_source}")
        self.get_logger().info(f"publish_labels filter: {sorted(list(self.publish_labels))}")
        self.get_logger().info(f"aliases_file: {self.aliases_file}")
        self.get_logger().info(f"target_class_ids: {self.target_class_ids}")
        self.get_logger().info(f"class_id_to_publish_label: {self.class_id_to_publish_label}")
        if self.unmatched_labels:
            self.get_logger().warn(
                f"unmatched whitelist labels: {self.unmatched_labels}. "
                "Check oiv7_aliases.json and oiv7_classes.list."
            )
        if self.publish_labels and not self.target_class_ids:
            self.get_logger().warn(
                "publish_labels is not empty, but no class id matched. "
                "Check oiv7_classes.list, semantic_mapping.yaml and oiv7_aliases.json."
            )

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
                target_class_ids=self.target_class_ids,
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
            raw_label = self.classes[cls_id] if 0 <= cls_id < len(self.classes) else f"class_{cls_id}"

            # 如果 class_id 是 alias 映射来的，就发布白名单里的名字；
            # 例如 raw_label=television，最终 label_norm=tv。
            label_norm = self.class_id_to_publish_label.get(
                cls_id,
                raw_label.strip().lower()
            )

            # 有 target_class_ids 时，前面已经只处理白名单类别了；
            # 没有 target_class_ids 时，才在这里兜底过滤。
            if self.publish_labels and not self.target_class_ids and label_norm not in self.publish_labels:
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
