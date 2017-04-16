Arm = function(ros) {
	var armExtendButton = document.querySelector('#armExtendButton');
	var armRelaxButton = document.querySelector('#armRelaxButton');

	var that = this;

	var setArmClient = new ROSLIB.Service({
		ros: ros,
		name: '/web_teleop/set_arm',
		serviceType: 'web_teleop/SetArm'
	});

	this.setArm = function(command) {
		var request = new ROSLIB.ServiceRequest({
			command: parseInt(command)
		});
		setArmClient.callService(request);
	}

	armExtendButton.addEventListener('click', function() {that.setArm(1);});
	armRelaxButton.addEventListener('click', function() {that.setArm(0);});
}
