ros2 launch rosbridge_server rosbridge_websocket_launch.xml

cd src/Joystick
python3 -m http.server 8000 & # ipv4

python3 -m http.server 8000 --bind :: # ipv6