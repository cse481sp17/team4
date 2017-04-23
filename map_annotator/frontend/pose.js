Pose = function(ros, name) {
  var that = this;
  this.name = name;

  var actionPub = new ROSLIB.Topic({
    ros: ros,
    name: '/user_actions',
    messageType: 'map_annotator/UserAction'
  });

  function handleGoTo() {
    console.log('Go to ' + name + ' clicked.');
    var msg = new ROSLIB.Message({
       command: "goto",
       name: name
    });
    actionPub.publish(msg)
  }

  function handleDelete() {
    console.log('Delete ' + name + ' clicked.');
    var msg = new ROSLIB.Message({
       command: "delete",
       name: name
    });
    actionPub.publish(msg)
  }

  this.render = function() {
    var node = document.createElement('div');
    var nameNode = document.createTextNode(name);
    node.appendChild(nameNode)

    var sendNode = document.createElement('input');
    sendNode.type = 'button';
    sendNode.value = 'Go to';
    sendNode.addEventListener('click', handleGoTo);
    node.appendChild(sendNode);

    var deleteNode = document.createElement('input');
    deleteNode.type = 'button';
    deleteNode.value = 'Delete';
    deleteNode.addEventListener('click', handleDelete);
    node.appendChild(deleteNode);
    return node;
  }
}