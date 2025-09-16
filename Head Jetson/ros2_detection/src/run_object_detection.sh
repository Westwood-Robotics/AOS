# run_ros2.sh

#!/bin/bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy,time;from std_msgs.msg import Bool;rclpy.init();\
from rclpy.node import Node; n=Node('multi_pub');p=n.create_publisher(Bool,'/smart_motion_trigger',10);\
m=Bool();m.data=False;[p.publish(m) or time.sleep(0.5) for _ in range(10)];n.destroy_node();rclpy.shutdown()"

pkill -9 -f object_detection_4
sleep 1
source /opt/ros/humble/setup.bash 

export PYTHONPATH=$PYTHONPATH:/home/nvidia/miniconda3/envs/groundedsam/lib/python3.10/site-packages
#export PYTHONPATH=/home/themis/miniconda3/envs/groundedsam/lib/python3.10/site-packages:$PYTHONPATH
source /opt/ros/humble/setup.bash
ros2 run object_detection object_detection_4  --depth_classifier_checkpoint ~/ros2_detection/weights/depth_size_classifier_szb.pth --filter_classes "s,z,b,s" --sam_checkpoint ~/ros2_detection/weights/mobile_sam.pt  --yolo_v8_checkpoint ~/ros2_detection/weights/yolov11n.pt  --obj_list_v8 "blue can,green can,red can,Plastic drinking bottle,storage box"


#ros2 run object_detection object_detection_4  --sam_checkpoint ~/ros2_detection/weights/mobile_sam.pt --yolo_v8_checkpoint ~/ros2_detection/weights/yolov11n.pt  --obj_list_v8 "blue can,green can,red can,Plastic drinking bottle,storage box" --depth_classifier_checkpoint ~/ros2_detection/weights/depth_size_classifier_szb.pth --filter_classes "s,z,b,s"

