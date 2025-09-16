安装object-detection功能包
cd ~ros2_detection/src
sudo dpkg -i ros-humble-object-detection_0.0.1-0jammy_arm64.deb


关于run_object_detection.sh中参数的介绍，run_object_detection.sh是启动检测脚本的sh文件，其中主要命令
ros2 run object_detection object_detection_4  --depth_classifier_checkpoint ~/ros2_zhuaqu/weights/depth_size_classifier_szb.pth --filter_classes "s,z,b,s" --sam_checkpoint ~/ros2_zhuaqu/weights/mobile_sam.pt  --yolo_v8_checkpoint ~/ros2_zhuaqu/weights/yolov11n.pt  --obj_list_v8 "blue can,green can,red can,Plastic drinking bottle,storage box"
可以根据自己电脑上的权重目录去修改。

python3 -c "import rclpy,time;from std_msgs.msg import Bool;rclpy.init();\
from rclpy.node import Node; n=Node('multi_pub');p=n.create_publisher(Bool,'/smart_motion_trigger',10);\
m=Bool();m.data=False;[p.publish(m) or time.sleep(0.5) for _ in range(10)];n.destroy_node();rclpy.shutdown()"这行代码是保证改脚本启动的时候杀死之前残留的相同的脚本，防止启动多个脚本
export PYTHONPATH=$PYTHONPATH:/home/nvidia/miniconda3/envs/groundedsam/lib/python3.10/site-packages临时进入虚拟环境
