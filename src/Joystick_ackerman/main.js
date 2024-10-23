const canvas = document.getElementById('joystickCanvas');
const context = canvas.getContext('2d');
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

const FPS = 60;

function clamp(value, min, max) {
    return Math.max(min, Math.min(value, max));
}

function background() {
    context.fillStyle = '#000';
    context.fillRect(0, 0, canvas.width, canvas.height);
}

// Create one joystick in the center of the screen
const joystick = new Joystick(canvas.width / 2, canvas.height / 2, 100, 50);

// ROS2 Connection
// const ros = new ROSLIB.Ros({
//     url: 'ws://192.168.31.206:9090'            ////// ipv4
// });
const ros = new ROSLIB.Ros({
    url: 'ws://[2409:8a28:8a0:aa20::3ce]:9090'    ////// ipv6
});

ros.on('connection', () => {
    console.log('Connected to ROS');
});

ros.on('error', (error) => {
    console.error('Error connecting to ROS:', error);
});

ros.on('close', () => {
    console.log('Disconnected from ROS');
});

const cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: 'cmd_vel_joy',       /// depending on motor driver topic, adjust it. 
    messageType: 'geometry_msgs/msg/Twist'
});

function sendCommand(linear, angular) {
    linear = isNaN(linear) ? 0 : linear;
    angular = isNaN(angular) ? 0 : angular;
    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
    });
    cmdVel.publish(twist);
}

setInterval(() => {
    background();

    joystick.update(context);
    // Map the joystick's Y position to linear speed range [-1.0, 1.0] m/s
    let traction_velocity_command = -((joystick.pos.y - joystick.origin.y) / joystick.radius);
    //console.log(joystick.pos.y, joystick.origin.y, joystick.radius )

    // Clamp the linearSpeed to ensure it stays within [-1.0, 1.0]
    traction_velocity_command = clamp(traction_velocity_command, -1.0, 1.0);

    // Map joystick x position to angular.z range [46, 130]
    let servo_position_command = -(joystick.pos.x - joystick.origin.x) / joystick.radius * (Math.PI / 4);
    //console.log(joystick.pos.x, joystick.origin.x, joystick.radius )

    // Clamp the angularSpeed to ensure it stays within [-0.785, 0.785]
    servo_position_command = clamp(servo_position_command, -Math.PI / 4, Math.PI / 4);
    console.log(`Linear: ${traction_velocity_command}, Angular: ${servo_position_command}`);

    
    sendCommand(traction_velocity_command, servo_position_command);

}, 1000 / FPS);
