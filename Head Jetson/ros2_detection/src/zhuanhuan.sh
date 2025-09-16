#!/bin/bash
# 设置 PYTHONPATH 环境变量
export PYTHONPATH=/home/nvidia/miniconda3/envs/groundedsam/lib/python3.10/site-packages:$PYTHONPATH

# 定义要查找的脚本名称
SCRIPT_NAME="zhuanhuan_entry.py"

# 查找运行 zhuanhuan.py 的进程 PID
PIDS=$(ps aux | grep "[p]ython3.*$SCRIPT_NAME" | awk '{print $2}')

# 检查是否找到进程
if [ -n "$PIDS" ]; then
    echo "Found existing $SCRIPT_NAME processes with PIDs: $PIDS"
    for PID in $PIDS; do
        echo "Terminating process with PID $PID..."
        kill -9 $PID 2>/dev/null
    done
    sleep 1  # 等待进程完全终止
else
    echo "No existing $SCRIPT_NAME processes found."
fi

# 启动 zhuanhuan.py
echo "Starting $SCRIPT_NAME..."
python3 ~/ros2_detection/src/zhuanhuan_entry.py
