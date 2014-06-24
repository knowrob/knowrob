/**
 * @author Arne Stefes - arne.stefes@gmail.com
 */

/**
 * A marker array client that listens to a given marker array topic and
 * passes markers from received arrays to a marker client
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker array topic to listen to
 *   * tfClient - the TF client handle to use
 *   * markerClient - the marker client handle to use
 */

ROS3D.MarkerArrayClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  var markerClient = options.markerClient;
  this.tfClient = options.tfClient;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/MarkerArray',
    compression : 'png'
  });

  // pass the markers in the array to the marker client
  rosTopic.subscribe(function(message) {
    for(var i = 0; i < message.markers.length; i++){
        markerClient.addMarker(message.markers[i]);
    }
  });
};
ROS3D.MarkerArrayClient.prototype.__proto__ = EventEmitter2.prototype;
