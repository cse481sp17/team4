Gripper = function(ros) {
  // HTML elements
  var desiredGripperForce = document.querySelector('#desiredGripperForce');
  var gripperSlider = document.querySelector('#gripperSlider');
  var gripperOpenButton = document.querySelector('#gripperOpenButton');
  var gripperCloseButton = document.querySelector('#gripperCloseButton');

  var that = this;

  var setGripperClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_gripper',
    serviceType: 'web_teleop/SetGripper'
  });

  
  // Initialize slider.
  var desiredForce = 35.0;
  desiredGripperForce.textContent = desiredForce;
  gripperSlider.value = desiredForce;

  // Update desiredHeight when slider moves.
  gripperSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredForce = gripperSlider.value;
    // Update the desired torso height display.
    desiredGripperForce.textContent = desiredForce;
  });

  // Method to set the height.
  this.setGrip = function(command, force) {
    var force = Math.min(Math.max(35.0, force), 100.0);
    var request = new ROSLIB.ServiceRequest({
      command: parseInt(command),
      force: parseFloat(force)
    });
    setGripperClient.callService(request);
  };

  gripperOpenButton.addEventListener('click', function() {
    that.setGrip(-1, desiredForce);
  });

  gripperCloseButton.addEventListener('click', function() {
    that.setGrip(0, desiredForce);
  });
}
