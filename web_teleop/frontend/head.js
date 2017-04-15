Head = function(ros) {
  // TODO: fix this.  backend node, actuator_nodes.py work.
  //       seen when called the service directly, issue should be here and in the .html
  // HTML elements
  var headTiltPosition = document.querySelector('#headTiltPosition');
  var desiredHeadTiltPosition = document.querySelector('#desiredHeadTiltPosition');
  var headTiltSlider = document.querySelector('#headTiltSlider');
  var headTiltButton = document.querySelector('#headTiltButton');

  var headPanPosition = document.querySelector('#headPanPosition');
  var desiredHeadPanPosition = document.querySelector('#desiredHeadPanPosition');
  var headPanSlider = document.querySelector('#headPanSlider');
  var headPanButton = document.querySelector('#headPanButton');
  desiredHeadTiltPosition.textContent = 0.5;

  var that = this;

  var setHeadClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_head',
    serviceType: 'web_teleop/SetHead'
  });

  // Listen to torso height from the joint_state_republisher.
  var listener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/head_pan_joint',
    messageType: 'std_msgs/Float64'
  });

  listener.subscribe(function(message) {
    // Whenever we get a message with a new torso height, update
    // the torso height display on the webpage.
    var pan = message.data;

    // Note the noise in the data. You can smooth it out using this line of code.
    pan = Math.round(pan*1000) / 1000
    headPanPosition.textContent = pan;
  });

  // Listen to torso height from the joint_state_republisher.
  var listener2 = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/head_tilt_joint',
    messageType: 'std_msgs/Float64'
  });

  listener2.subscribe(function(message) {
    // Whenever we get a message with a new torso height, update
    // the torso height display on the webpage.
    var tilt = message.data;

    // Note the noise in the data. You can smooth it out using this line of code.
    tilt = Math.round(tilt*1000) / 1000
    headTiltPosition.textContent = tilt;
  });
  
  // Initialize head tilt slider.
  var desiredTilt = 0.0;
  desiredHeadTiltPosition.textContent = desiredTilt;
  headTiltSlider.value = desiredTilt;

  // Initialize head tilt slider.
  var desiredPan = 0.0;
  desiredHeadPanPosition.textContent = desiredPan;
  headPanSlider.value = desiredPan;

  // Update desiredHeadPan when slider moves.
  headPanSlider.addEventListener('input', function() {
    desiredPan = headPanSlider.value;
    desiredHeadPanPosition.textContent = desiredPan;
  });

  // Update desiredHeadTilt when slider moves.
  headTiltSlider.addEventListener('input', function() {
    desiredTilt = headTiltSlider.value;
    desiredHeadTiltPosition.textContent = desiredTilt;
  });


  // Method to set the head pan and tilt.
  this.setHead = function(tilt, pan) {
    //var tilt = Math.min(Math.max(-Math.pi/2, tilt), Math.pi/4);
    var request = new ROSLIB.ServiceRequest({
	  tilt: parseFloat(tilt),
	  pan: parseFloat(pan)
    });
    setHeadClient.callService(request);
  };

  // Set the height when the button is clicked.
  headPanButton.addEventListener('click', function() {
	// never gets here, problem somewher einbetween
	//that.setHead(1, 1);
	//desiredHeadTiltPosition.textContent = desiredPan;
	that.setHead(desiredTilt, desiredPan);
  });

  headTiltButton.addEventListener('click', function() {
	//that.setHead(1, 1);
    that.setHead(desiredTilt, desiredPan);
  });
}
