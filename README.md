本项目需求第三方库：
realsense2
OpenCV
Python3
pybind11
Threads
CURL
json

运行前终端需要设置根路径
export ROBOT_HOME=/media/ubuntu/3063-3833/Program/Project/lht_fr/dhu_frobot-main
如果机械臂ip连接失败需要重新设置ip
sudo ip addr flush dev eth01
sudo ip link set eth01 up
sudo ip addr add 192.168.1.10/24 dev eth01
ping 192.168.1.100 //机械臂ip

