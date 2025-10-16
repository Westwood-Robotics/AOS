#ros_api relevant bash script in the service that performs the detection of Head Jetsond
You can change the real IP of Head Jetsond in the corresponding script and set it to 192.168.0.11
Pass
```bash
sshpass -p nvidia ssh nvidia@192.168.0.72 'echo nvidia | ./ros2_detection/src/run_object_detection.sh'
```
Implement the execution of commands
