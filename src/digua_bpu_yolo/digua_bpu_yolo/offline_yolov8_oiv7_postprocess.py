#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np


OUTPUT_SPECS = [
    # cls_file, reg_file, grid_h, grid_w, stride
    ("model_infer_output_0_output0.bin", "model_infer_output_1_326.bin", 80, 80, 8),
    ("model_infer_output_2_334.bin", "model_infer_output_3_342.bin", 40, 40, 16),
    ("model_infer_output_4_350.bin", "model_infer_output_5_358.bin", 20, 20, 32),
]


def sigmoid(x):
    x = np.clip(x, -50, 50)
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x, axis=-1):
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / np.sum(e, axis=axis, keepdims=True)


def read_float32(path, shape):
    arr = np.fromfile(path, dtype=np.float32)
    expected = int(np.prod(shape))
    if arr.size != expected:
        raise RuntimeError(
            f"bad tensor size: {path}, got {arr.size}, expected {expected}, shape={shape}"
        )
    return arr.reshape(shape)


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

    # auto:
    # 如果分类输出已经在 0~1 内，直接用；
    # 如果有负数或大于 1，按 logits 处理，走 sigmoid。
    mn = float(np.min(cls))
    mx = float(np.max(cls))
    if mn < 0.0 or mx > 1.0:
        return sigmoid(cls)
    return cls


def decode_one_level(cls, reg, stride, score_threshold, cls_mode):
    """
    cls: H,W,C
    reg: H,W,64
    return boxes_xyxy, scores, class_ids
    """
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

    reg_keep = reg[ys, xs, :]              # N,64
    reg_keep = reg_keep.reshape(-1, 4, 16) # N,4,16

    prob = softmax(reg_keep, axis=-1)
    bins = np.arange(16, dtype=np.float32)
    dist = np.sum(prob * bins, axis=-1)    # N,4

    # YOLOv8 anchor point: grid cell center
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


def print_tensor_stats(name, arr):
    flat = arr.reshape(-1)
    print(
        f"[stats] {name}: shape={arr.shape}, "
        f"min={float(np.min(flat)):.6f}, "
        f"max={float(np.max(flat)):.6f}, "
        f"mean={float(np.mean(flat)):.6f}"
    )


def draw_preview(image_path, detections, out_path):
    img = cv2.imread(image_path)
    if img is None:
        print(f"[warn] failed to read image: {image_path}")
        return

    img_h, img_w = img.shape[:2]

    # 这里先按“模型输入 640x640 坐标线性缩放到原图”画预览。
    # 如果你实际推理使用 letterbox，后面实时节点里会单独处理坐标还原。
    sx = img_w / 640.0
    sy = img_h / 640.0

    for det in detections:
        x1, y1, x2, y2 = det["bbox_xyxy_model"]
        x1 = int(x1 * sx)
        x2 = int(x2 * sx)
        y1 = int(y1 * sy)
        y2 = int(y2 * sy)

        label = det["label"]
        score = det["confidence"]

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            img,
            f"{label} {score:.2f}",
            (x1, max(0, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, img)
    print(f"[ok] wrote preview: {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump_dir", default="/home/sunrise/rdk_normal_yolo_test/dump")
    parser.add_argument("--image", default="/home/sunrise/rdk_normal_yolo_test/calib_040.jpg")
    parser.add_argument("--classes", default="/home/sunrise/digua_ws/src/digua_bpu_yolo/config/oiv7_classes.list")
    parser.add_argument("--score_threshold", type=float, default=0.25)
    parser.add_argument("--nms_threshold", type=float, default=0.7)
    parser.add_argument("--top_k", type=int, default=100)
    parser.add_argument("--cls_mode", choices=["auto", "sigmoid", "raw"], default="auto")
    parser.add_argument("--output_json", default="/home/sunrise/rdk_normal_yolo_test/result/digua_oiv7_offline_detections.json")
    parser.add_argument("--output_image", default="/home/sunrise/rdk_normal_yolo_test/result/digua_oiv7_offline_preview.jpg")
    args = parser.parse_args()

    dump_dir = Path(args.dump_dir)
    classes = load_classes(args.classes)
    print(f"[info] classes: {len(classes)}")
    print(f"[info] score_threshold: {args.score_threshold}")
    print(f"[info] nms_threshold: {args.nms_threshold}")
    print(f"[info] cls_mode: {args.cls_mode}")

    all_boxes = []
    all_scores = []
    all_class_ids = []

    for cls_file, reg_file, h, w, stride in OUTPUT_SPECS:
        cls_path = dump_dir / cls_file
        reg_path = dump_dir / reg_file

        cls = read_float32(cls_path, (h, w, 601))
        reg = read_float32(reg_path, (h, w, 64))

        print_tensor_stats(cls_file, cls)
        print_tensor_stats(reg_file, reg)

        boxes, scores, class_ids = decode_one_level(
            cls=cls,
            reg=reg,
            stride=stride,
            score_threshold=args.score_threshold,
            cls_mode=args.cls_mode,
        )

        print(f"[level stride={stride}] candidates after score filter: {len(scores)}")

        all_boxes.append(boxes)
        all_scores.append(scores)
        all_class_ids.append(class_ids)

    boxes = np.concatenate(all_boxes, axis=0)
    scores = np.concatenate(all_scores, axis=0)
    class_ids = np.concatenate(all_class_ids, axis=0)

    print(f"[info] total candidates before nms: {len(scores)}")

    if len(scores) == 0:
        detections = []
    else:
        keep = nms_class_aware(
            boxes=boxes,
            scores=scores,
            class_ids=class_ids,
            iou_threshold=args.nms_threshold,
            top_k=args.top_k,
        )

        detections = []
        for i in keep:
            cls_id = int(class_ids[i])
            label = classes[cls_id] if 0 <= cls_id < len(classes) else f"class_{cls_id}"

            x1, y1, x2, y2 = [float(v) for v in boxes[i]]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = x2 - x1
            bh = y2 - y1

            detections.append({
                "label": label,
                "class_id": cls_id,
                "confidence": float(scores[i]),
                "bbox": {
                    "cx": cx,
                    "cy": cy,
                    "w": bw,
                    "h": bh
                },
                "bbox_xyxy_model": [x1, y1, x2, y2]
            })

    out = {
        "frame_id": "camera_color_optical_frame",
        "image_width_model": 640,
        "image_height_model": 640,
        "detections": detections,
    }

    os.makedirs(os.path.dirname(args.output_json), exist_ok=True)
    with open(args.output_json, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    print(f"[ok] wrote json: {args.output_json}")
    print(f"[info] final detections: {len(detections)}")

    for det in detections[:20]:
        print(
            f"[det] {det['label']} "
            f"score={det['confidence']:.4f} "
            f"bbox={det['bbox']}"
        )

    draw_preview(args.image, detections, args.output_image)


if __name__ == "__main__":
    main()
