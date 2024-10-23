!/bin/bash


# Start the RACECAR launch
echo "STARTING RACECAR"
ros2 launch racecar_bringup racecar.launch.py &


# Wait for a moment to ensure the first launch file is running
sleep 5

# Start the NAVIGATION launch
echo "STARTING NAVIGATION"
ros2 ros2 launch racecar_bringup nav2_bringup.py &




# Start the ROSBridge WebSocket server launch
# echo "Starting ROSBridge WebSocket server launch..."
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
# ROSBRIDGE_PID=$!

# Serve the HTML file using Python's built-in HTTP server
# echo "Serving HTML file on localhost..."
# cd src/Joystick_ackerman
# python3 -m http.server 8000 & # ipv4

# python3 -m http.server 8000 --bind :: # ipv6
# HTTP_SERVER_PID=$!

# Provide information to the user
# echo "HTML file is being served at http://localhost:8000/index.html"


# Nav2 launch
#ros2 launch nav2_bringup bringup_launch.py params_file:=/home/car/racecarROS2/src/racecar_driver_hardware/config/nav2_params.yaml map:=/home/car/racecarROS2/src/racecar_driver_hardware/config/map.yaml.yaml

#ros2 launch racecar_bringup nav2_bringup.py map:=/home/car/racecarROS2/install/racecar_bringup/share/racecar_bringup/Map/map.yaml
echo "All tasks have been completed."
