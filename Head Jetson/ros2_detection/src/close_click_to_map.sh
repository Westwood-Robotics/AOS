#!/bin/bash
# kill_ros2_click_to_map.sh
# 查找并关闭运行的 click_to_map_converter.py 节点

echo "Stopping click_to_map_converter.py..."
pkill -f "click_to_map_converter_entry.py"

# 等待一下，确认杀掉
sleep 1
if pgrep -f "click_to_map_converter_entry.py" > /dev/null; then
    echo "❌ Failed to stop click_to_map_converter.py"
else
    echo "✅ Node stopped successfully"
fi
