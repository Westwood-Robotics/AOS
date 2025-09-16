#!/bin/bash
echo nvidia | sudo -S rdate -s 192.168.0.71

sleep 1s

echo nvidia | sudo -S systemctl restart zedxm_camera.service

