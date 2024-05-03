#!/usr/bin/env python3

"""Yolo RT Node"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import torch
from models.experimental import attempt_load
from models.common import non_max_suppression

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, BoundingBox2D, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis

from threading import Lock

class_list = [ 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
         'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
         'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
         'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
         'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
         'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
         'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
         'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
         'hair drier', 'toothbrush' ]

import numpy as np
class Yolo_ST(Node):
    """Yolo RT Node"""

    x = None  # input of the next layer
    y = []  # outputs of layers
    stream = torch.cuda.default_stream()
    output = None
    start_time = None
    finish_time = None
    cuda_device = torch.cuda.current_device() if torch.cuda.is_available() else 'cpu'
    torch_device = torch.device(cuda_device)
    timer_start_time = None
    timer_finish_time = None
    finished = True

    def __init__(self):
        """Initializes the node"""
        super().__init__('yolo_node')
        rclpy.logging._root_logger.info('yolo_node started')

        self.declare_parameter('weights', 'weights/yolov7-tiny.pt')
        weights_path = self.get_parameter('weights').value
        rclpy.logging._root_logger.info('weights_path: ' + weights_path)

        self.model = attempt_load(weights_path, map_location=self.torch_device)  # load FP32 model
        self.model.eval()
        self.model.half()

        self.create_timer(0.0, self.forward_one)
        rclpy.logging._root_logger.info('yolo_node initialized')

        # self.create_timer(1, self.image_callback)

        # create a subscription to /camera
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # create a bounding box publisher
        self.bbox_publisher = self.create_publisher(Detection2DArray, '/detections', 10)

    def image_callback(self, msg):
        # start new forward
        if not self.finished:
            self._logger.info('Image received and rejected')
            return
        
        self._logger.info('Image received')

        # torch.save(np.frombuffer(msg.data, dtype=np.uint8), "rawimage.pt")

        self.timer_start_time = self.get_clock().now()

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        torch.save(img, "image.pt")

        # convert msg data with width and height to tensor
        x = torch.from_numpy(img).permute(2, 0, 1).to(self.torch_device).float().half()
        x = x.unsqueeze(0)

        # # resize image to half
        x = torch.nn.functional.interpolate(x, size=(msg.height // 2, msg.width // 2), mode='bilinear', align_corners=False)

        # # crop to multiple of 32
        x = x[:, :, :352, :640]

        # normalize
        x /= 255.0

        if self.finished:
            self.cancel_and_start_new(x)

    def cancel_and_start_new(self, x):
        layer = len(self.y)
        last = self.y[-1] if len(self.y) > 0 else None
        self._logger.info("Terminated at layer: " + str(len(self.y)))
        self.x = x
        self.y = []
        self.finished = False
        return layer, last


    def convert(self, z):
        box = z[:, :, :4]
        conf = z[:, :, 4:5]
        score = z[:, :, 5:]
        score *= conf
        convert_matrix = torch.tensor([[1, 0, 1, 0], [0, 1, 0, 1], [-0.5, 0, 0.5, 0], [0, -0.5, 0, 0.5]],
                                           dtype=torch.half,
                                           device=z.device)
        box @= convert_matrix                          
        return (box, score)
        

    def forward_one(self):
        if self.stream.query() and not self.finished:
            if len(self.y) == len(self.model.model):  # finished?
                # NMS
                pred = non_max_suppression(self.y[-1][0], conf_thres=0.25, iou_thres=0.45, classes=None)[0]


                # create and fill the detection 2d array msg
                msg = Detection2DArray()
                header = msg.header
                header.frame_id = "camera"
                header.stamp = self.get_clock().now().to_msg()


                for i in range(pred.shape[0]): 

                    min_x = pred[i][0]
                    min_y = pred[i][1]
                    max_x = pred[i][2]
                    max_y = pred[i][3]
                    score = pred[i][4]
                    class_id = pred[i][5]

                    detection = Detection2D()
                    bbox = BoundingBox2D()
                    objecthypothesiswithpose = ObjectHypothesisWithPose()
                    objecthypothesis = ObjectHypothesis()

                    bbox.center.x = float((min_x + max_x)/2)*2
                    bbox.center.y = float((min_y + max_y)/2)*2
                    bbox.size_x = float(max_x - min_x)*2
                    bbox.size_y = float(max_y - min_y)*2

                    detection.bbox = bbox

                    objecthypothesis.class_id = str(class_list[int(class_id)])
                    objecthypothesis.score = float(score)

                    objecthypothesiswithpose.hypothesis = objecthypothesis

                    detection.results.append(objecthypothesiswithpose)

                    msg.detections.append(detection)

                # publish the detection 2d array msg
                self.bbox_publisher.publish(msg)

                self.finished = True
                self.timer_finish_time = self.get_clock().now()

                if self.timer_start_time is not None and self.timer_finish_time is not None:
                    # print in seconds
                    self._logger.info('Time difference: ' + str((self.timer_finish_time - self.timer_start_time).nanoseconds / 1000000000))
                
                self._logger.info("Detected objects: " + str(pred))
                return
            with torch.no_grad():
                m = self.model.model[len(self.y)]  # get layer
                if m.f != -1:  # if not from previous layer
                    self.x = self.y[m.f] if isinstance(m.f, int) else [self.x if j == -1 else self.y[j] for j in m.f]  # from earlier layers
                self.x = m(self.x)  # run layer, next x is output of current layer
                self.y.append(self.x)  # append output


class YOLO_MT(Node):
    """
    YOLO MT Node
    """

    finished = True
    mutex = Lock()

    def __init__(self):
        """Initializes the node"""
        super().__init__('yolo_node')
        rclpy.logging._root_logger.info('yolo_node started')

        # create two callback groups
        self.callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.callback_group2 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.cuda_device = torch.cuda.current_device() if torch.cuda.is_available() else 'cpu'
        self.torch_device = torch.device(self.cuda_device)

        self.x = None
        self.y = []

        self.output = None

        self.forwarding = False
        
        self.declare_parameter('weights', 'weights/yolov7-tiny.pt')
        weights_path = self.get_parameter('weights').value
        self.model = attempt_load(weights_path, map_location=self.torch_device)  # load FP32 model
        self.model.eval()
        self.model.half()

        self.cancel_queued = False

        self.start_queued = False
        
        print(len(self.model.model))

        # create a timer for forward
        self.create_timer(0.0, self.forward, callback_group=self.callback_group)

        # create a subscription for images
        # self.create_subscription(
        #     Image,
        #     '/camera/color/image_raw',
        #     self.image_callback,
        #     10,
        #     callback_group=self.callback_group2
        # )

        self.create_timer(0.1, self.image_callback, callback_group=self.callback_group2)

    def image_callback(self):
                    #    , msg):
        # start new forward
        self._logger.info('Image received')
        
        x = torch.randn(1, 3, 640, 640).to(self.torch_device).half()
      
        # publish self.y[-1]

        self.mutex.acquire()
        self.start_queued = True
        self.cancel_queued = True
        self.x = x
        self.mutex.release()

    def convert(self, z):
        box = z[:, :4]
        conf = z[:, 4:5]
        score = z[:, 5:]
        score *= conf
        convert_matrix = torch.tensor([[1, 0, 1, 0], [0, 1, 0, 1], [-0.5, 0, 0.5, 0], [0, -0.5, 0, 0.5]],
                                           dtype=torch.float32,
                                           device=z.device)
        box @= convert_matrix                          
        return (box, score)
    
    def forward(self):
        if not self.forwarding and self.start_queued:
            self._logger.info('Forwarding')
            self.forwarding = True
            self.mutex.acquire()
            self.start_queued = False
            self.cancel_queued = False

            x = self.x
            self.mutex.release()

            timer_start = self.get_clock().now()

            with torch.no_grad():
                while len(self.y) != len(self.model.model) and not self.cancel_queued:
                    m = self.model.model[len(self.y)] # get layer
                    if m.f != -1: # if not from previous layer
                        x = self.y[m.f] if isinstance(m.f, int) else [x if j == -1 else self.y[j] for j in m.f]
                    x = m(x)
                    self.y.append(x)
                    if len(self.y) % 10 == 0:
                        torch.cuda.synchronize()
            torch.cuda.synchronize()

            timer_end = self.get_clock().now()
            self._logger.info('Time difference: ' + str((timer_end - timer_start).nanoseconds / 1000000000))
            self._logger.info('Terminated at layer: ' + str(len(self.y)))

            # output = None

            # if len(self.y) == len(self.model.model):
            #     output = self.y[-1]

            self.y = []

            # if output is not None:
            #     nms_timer_start = self.get_clock().now()
            #     nms_result = non_max_suppression(output[0], conf_thres=0.25, iou_thres=0.45, classes=None)[0].cpu()
            #     torch.cuda.synchronize()
            #     nms_timer_end = self.get_clock().now()
            #     self._logger.info('NMS time difference: ' + str((nms_timer_end - nms_timer_start).nanoseconds / 1000000000))
            #     self._logger.info("Detected objects: " + str(nms_result))

            self.forwarding = False

def main(args=None):
    """Main function"""

    if sys.argv[1] == 'single':
        rclpy.init(args=args)
        node = Yolo_ST()
        rclpy.spin(node)
        rclpy.shutdown()
    elif sys.argv[1] == 'multi':
        rclpy.init(args=args)
        node = YOLO_MT()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()

        rclpy.shutdown()
    elif sys.argv[1] == "timing_test2":
        print('Timing test')
        model = attempt_load("/home/f110xaviernx/ros2_rt_yolo/src/yolo_rt/weights/yolov7-tiny.pt")
        model = model.half()
        model.eval()
        # import torch.utils.benchmark as benchmark
        # t0 = benchmark.Timer(
        #     stmt = """
        #     with torch.no_grad():
        #         y = []
        #         x = img
        #         for m in model.model:
        #             if m.f != -1:
        #                 x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]
        #             x = m(x)
        #             y.append(x)
        #             torch.cuda.synchronize()
        #     """,
        #     setup = "img = torch.randn(1, 3, 640, 640).cuda().half()",
        #     globals = { 'model': model }
        # )
    elif sys.argv[1] == "timing_test":
        print('Timing test')
        model = attempt_load("/home/f110xaviernx/ros2_rt_yolo/src/yolo_rt/weights/yolov7-tiny.pt")
        model = model.half()
        model.eval()
        # model.model = model.model[:-1]

        import torch.utils.benchmark as benchmark
        t0 = benchmark.Timer(
            stmt = "with torch.no_grad(): output = model(img)",
            setup = "img = torch.randn(1, 3, 640, 640).cuda().half()",
            globals = { 'model': model }
        )

        print(t0.timeit(1000))
    elif sys.argv[1] == "timing_test_nms":
        print("Timing test NMS")
        model = attempt_load("/home/f110xaviernx/ros2_rt_yolo/src/yolo_rt/weights/yolov7-tiny.pt")
        model.half()
        model.eval()
        import torch.utils.benchmark as benchmark
        
        t0 = benchmark.Timer(
            stmt = "nms_result = non_max_suppression(output[0], conf_thres=0.25, iou_thres=0.45, classes=None)[0].cpu()",
            setup = """
            img = torch.randn(1, 3, 640, 640).cuda().half()
            with torch.no_grad():
                output = model(img)
            output = (output[0].cpu(), )
            """,
            globals = { 'model': model, 'non_max_suppression': non_max_suppression }
        )

        print(t0.timeit(1000))
    else:
        print('Please specify multithreaded or single_threaded')

if __name__ == '__main__':
    main()