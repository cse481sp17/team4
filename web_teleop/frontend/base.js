Base = function(ros) {
  // HTML elements
  // To get an element with an ID of "baseForward", query it as shown below.
  // Note that any IDs you set on HTML elements should be unique.
  var baseForward = document.querySelector('#baseForward');
  var baseBackward = document.querySelector('#baseBackward');
  var baseRight = document.querySelector('#baseRight');
  var baseLeft = document.querySelector('#baseLeft');

  var that = this;

  // Public variables (outsiders can set this using base.linearSpeed = 0.1)
  this.linearSpeed = 0.40;
  this.angularSpeed = 0.40;

  // Set up the publisher.
  var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });

  // Internal function to send a velocity command.
  var move = function(linear, angular) {
    var twist = new ROSLIB.Message({
      linear: {
        x: linear,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angular
      }
    });  
    cmdVel.publish(twist);
  }

  // Handler for when the mouse is held on the up arrow.
  // Instead of writing a loop (which will block the web page), we use
  // setInterval, which repeatedly calls the given function at a given
  // time interval. In this case, it repeatedly calls move() every 50 ms.
  // Note that inside of move, we use that._timer and that.linearSpeed.
  // At the top of the file we set "var that = this" to ensure that the
  // local variable "that" always refers to this Base instance.
  this.moveForward = function() {
    that._timer = setInterval(function() {
      move(that.linearSpeed, 0)
    }, 50);
  }

  this.moveBackward = function() {
    that._timer = setInterval(function() {
      move(-that.linearSpeed, 0)
    }, 50);
  }

  this.turnRight = function() {
    that._timer = setInterval(function() {
      move(0, -that.angularSpeed)
    }, 50);
  }

  this.turnLeft = function() {
    that._timer = setInterval(function() {
      move(0, that.angularSpeed)
    }, 50);
  }

  // Stops the robot from moving.
  this.stop = function() {
    if (that._timer) {
      clearInterval(that._timer);
    }
    move(0, 0);
  };  

  baseForward.addEventListener('mousedown', that.moveForward);
  baseBackward.addEventListener('mousedown', that.moveBackward);
  baseRight.addEventListener('mousedown', that.turnRight);
  baseLeft.addEventListener('mousedown', that.turnLeft);
  
  // We bind stop() to whenever the mouse is lifted up anywhere on the webpage
  // for safety reasons. We want to be conservative about sending movement commands.
  document.addEventListener('mouseup', that.stop);
}


