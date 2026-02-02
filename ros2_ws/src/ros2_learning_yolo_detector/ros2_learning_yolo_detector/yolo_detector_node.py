#!/usr/bin/env python3
"""
YOLO v8 目标检测 ROS 2 节点

功能说明:
=========
这个节点订阅相机图像，使用 YOLO v8 进行目标检测，然后发布检测结果。

话题:
=====
- 订阅: /camera/image_raw (sensor_msgs/Image) - 输入图像
- 发布: /detections (vision_msgs/Detection2DArray) - 检测结果
- 发布: /detection_image (sensor_msgs/Image) - 标注后的图像

参数:
=====
- model_name: YOLO 模型名称 (默认: yolov8n.pt，最小最快)
- confidence_threshold: 置信度阈值 (默认: 0.5)
- image_topic: 输入图像话题 (默认: /camera/image_raw)

使用方法:
=========
1. 安装依赖: pip install ultralytics
2. 运行节点: ros2 run ros2_learning_yolo_detector yolo_detector
3. 或使用 launch 文件: ros2 launch ros2_learning_yolo_detector yolo_detector.launch.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2

# 延迟导入 ultralytics，避免启动时报错
YOLO = None


def load_yolo():
    """延迟加载 YOLO 模块"""
    global YOLO
    if YOLO is None:
        try:
            from ultralytics import YOLO as _YOLO
            YOLO = _YOLO
        except ImportError:
            raise ImportError(
                "请安装 ultralytics: pip install ultralytics"
            )
    return YOLO


class YoloDetectorNode(Node):
    """
    YOLO 目标检测节点

    这个节点展示了如何将深度学习模型集成到 ROS 2 中:
    1. 订阅相机图像
    2. 使用 YOLO 进行推理
    3. 发布标准的 vision_msgs 检测结果
    """

    def __init__(self):
        super().__init__('yolo_detector')

        # ========== 声明参数 ==========
        # ROS 2 的参数系统允许在运行时动态配置节点
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')

        # 获取参数值
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # ========== 加载 YOLO 模型 ==========
        self.get_logger().info(f'正在加载 YOLO 模型: {model_name}')
        try:
            YOLO_class = load_yolo()
            self.model = YOLO_class(model_name)
            self.get_logger().info('YOLO 模型加载成功!')
        except Exception as e:
            self.get_logger().error(f'加载模型失败: {e}')
            raise

        # ========== 创建 cv_bridge ==========
        # cv_bridge 用于在 ROS Image 消息和 OpenCV 图像之间转换
        self.bridge = CvBridge()

        # ========== 创建订阅者 ==========
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10  # QoS 队列深度
        )

        # ========== 创建发布者 ==========
        # 发布检测结果 (结构化数据)
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        # 发布标注后的图像 (用于可视化)
        self.image_pub = self.create_publisher(
            Image,
            '/detection_image',
            10
        )

        self.get_logger().info(f'YOLO 检测节点已启动!')
        self.get_logger().info(f'  - 订阅图像话题: {image_topic}')
        self.get_logger().info(f'  - 置信度阈值: {self.conf_threshold}')

    def image_callback(self, msg: Image):
        """
        图像回调函数

        处理流程:
        1. 将 ROS Image 转换为 OpenCV 格式
        2. 调用 YOLO 进行检测
        3. 解析检测结果
        4. 发布检测结果和标注图像
        """
        try:
            # 1. ROS Image → OpenCV (BGR 格式)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 2. 运行 YOLO 检测
        # verbose=False 禁止在控制台打印每帧的检测信息
        results = self.model(cv_image, verbose=False, conf=self.conf_threshold)

        # 3. 解析结果并构建 Detection2DArray 消息
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # YOLO 返回一个结果列表，我们只处理第一个 (单张图像)
        if len(results) > 0:
            result = results[0]

            # 遍历每个检测框
            for box in result.boxes:
                detection = Detection2D()

                # 获取边界框坐标 (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Detection2D 使用中心点 + 尺寸表示
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # 添加分类结果
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.model.names[int(box.cls[0])]
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                detection_array.detections.append(detection)

            # 4. 绘制检测结果到图像上
            annotated_image = result.plot()  # YOLO 内置的绘图函数

        else:
            annotated_image = cv_image

        # 5. 发布检测结果
        self.detection_pub.publish(detection_array)

        # 6. 发布标注后的图像
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'发布图像失败: {e}')

        # 日志 (仅在有检测结果时输出)
        if len(detection_array.detections) > 0:
            self.get_logger().debug(
                f'检测到 {len(detection_array.detections)} 个目标'
            )


def main(args=None):
    """节点入口点"""
    rclpy.init(args=args)

    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点异常: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
