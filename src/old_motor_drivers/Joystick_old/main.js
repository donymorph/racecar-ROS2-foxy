const canvas = document.getElementById('joystickCanvas');
const context = canvas.getContext('2d');
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

const FPS = 60;

function background() {
    context.fillStyle = '#000';
    context.fillRect(0, 0, canvas.width, canvas.height);
}

// Create one joystick in the center of the screen
const joystick = new Joystick(canvas.width / 2, canvas.height / 2, 100, 50);

// ROS2 Connection
const ros = new ROSLIB.Ros({
    url: 'ws://192.168.31.206:9090'         /// ipv4
});

// const ros = new ROSLIB.Ros({
//     url: 'ws://[2409:8a28:8a0:aa20::3ce]:9090'    ////// ipv6
// });
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
    name: '/car/cmd_vel',                   /// depending on motor driver topic, adjust it.
    messageType: 'geometry_msgs/Twist'
});

function sendCommand(linear, angular) {
    linear = isNaN(linear) ? 1500 : linear;
    angular = isNaN(angular) ? 84 : angular;
    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
    });
    cmdVel.publish(twist);
}

setInterval(() => {
    background();

    joystick.update(context);
    // Map the joystick's Y position to linear speed range [1360, 1580]
    let linearSpeed = 1500 - ((joystick.pos.y - joystick.origin.y) / joystick.radius) * (1500 - 1200);
    console.log(joystick.pos.y, joystick.origin.y, joystick.radius )
    // Clamp the linearSpeed to ensure it stays within [1360, 1580]
    linearSpeed = Math.max(1200, Math.min(1600, linearSpeed));

    // Map joystick x position to angular.z range [46, 130]
    let angularSpeed = 84 - (joystick.pos.x - joystick.origin.x) / joystick.radius * (130 - 46);
    console.log(joystick.pos.x, joystick.origin.x, joystick.radius )
    angularSpeed = Math.max(46, Math.min(130, angularSpeed));
    console.log(`Linear: ${linearSpeed}, Angular: ${angularSpeed}`);
    sendCommand(linearSpeed, angularSpeed);

}, 1000 / FPS);
