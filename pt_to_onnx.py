from ultralytics import YOLO

def main():
    pt_path = "yolov8n.pt"
    model = YOLO(pt_path)

    # 建议 imgsz=512（CPU上更快），后面ROS节点也用同样的 imgsz
    model.export(
        format="onnx",
        imgsz=448,
        opset=12,
        simplify=True,
        dynamic=False,   # 固定输入尺寸，CPU上通常更快更稳
        nms=True,        # 关键：导出带NMS的 end-to-end ONNX
        conf=0.25,
        iou=0.45,
        max_det=50,
    )

if __name__ == "__main__":
    main()
