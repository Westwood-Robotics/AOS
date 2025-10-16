# Head Jetson Documentation
## Install the ros2_detection package
```bash
cd ~ros2_detection/src
sudo dpkg -i ros-humble-object-detection_0.0.1-0jammy_arm64.deb
```


Regarding the introduction of the parameters in the run_object_detection.sh, run_object_detection.sh is the sh file that starts the detection script, where the main command is
```bash
ros2 run object_detection object_detection_4  --depth_classifier_checkpoint ~/ros2_detection/weights/depth_size_classifier_szb.pth --filter_classes "s,z,b,s" --sam_checkpoint ~/ ros2_detection/weights/mobile_sam.pt  --yolo_v8_checkpoint ~/ros2_detection/weights/yolov11n.pt  --obj_list_v8 "blue can,green can,red can,Plastic drinking bottle,storage box"
```
You can modify it according to the weight directory on your computer.

```bash
python3 -c "import rclpy,time; from std_msgs.msg import Bool; rclpy.init(); from rclpy.node import Node; n=Node('multi_pub'); p=n.create_publisher(Bool,'smart_motion_trigger',10); m=Bool(); m.data=False; [p.publish(m) or time.sleep(0.5) for _ in range(10)]; n.destroy_node(); rclpy.shutdown()"
```
This line of code is to ensure that the script is started to kill the same script that remained before it, preventing multiple scripts from starting
export PYTHONPATH=$PYTHONPATH:/home/nvidia/miniconda3/envs/groundedsam/lib/python3.10/site-packages temporarily enter the virtual environment


## Environment configuration
```bash
pip install ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple --timeout 100
pip install numpy==1.26.4
pip install timm -i https://pypi.tuna.tsinghua.edu.cn/simple
cd
git clone git@github.com:ChaoningZhang/MobileSAM.git
cd MobileSAM; pip install -e .
```
