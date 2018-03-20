/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Nils Berg - berg.nils@gmail.com
 */

/**
 * A MarkerArray client that listens to a given topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the MarkerArray
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 *   * tfClient - the TF client handle to use
 *   * sceneObjects (optional) - the root object to add the markers to
 *   * selectableObjects (optional) - the root object to add the selectable markers to
 *   * backgroundObjects (optional) - the root object to add the background markers to
 *   * path (optional) - the base path to any meshes that will be loaded
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.MarkerArrayClient = function(options) {
  var that = this;
  
  options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  this.tfClient = options.tfClient;
  this.selectableObjects = options.selectableObjects || new THREE.Object3D();
  this.sceneObjects = options.sceneObjects || new THREE.Object3D();
  this.backgroundObjects = options.backgroundObjects || new THREE.Object3D();
  this.orthogonalObjects = options.orthogonalObjects || new THREE.Object3D();
  this.path = options.path || '/';
  this.loader = options.loader || ROS3D.COLLADA_LOADER_2;
  this.on_dblclick = options.on_dblclick || function(_) { };
  this.on_contextmenu = options.on_contextmenu || function(_) { };
  this.on_delete = options.on_delete || function(_) { };
  
  this.setMaxListeners(0);
  
  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};

  // subscribe to MarkerArray topic
  var arrayTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/MarkerArray',
    compression : 'png'
  });
  
  var markerScene = function(m) {
      if(m.isBackgroundMarker) { return that.backgroundObjects; }
      else if(m.isSelectable) { return that.selectableObjects; }
      else if(m.isSceneOrtho) { return that.orthogonalObjects; }
      else { return that.sceneObjects; }
  };
  
  arrayTopic.subscribe(function(arrayMessage) {

    arrayMessage.markers.forEach(function(message) {
      if(message.action === 0) {
        var updated = false;
        if(message.ns + message.id in that.markers) { // MODIFY
          var m = that.markers[message.ns + message.id];
          updated = m.children[0].update(message);
          if(!updated) { // REMOVE
              m.unsubscribeTf();
              markerScene(m.object).remove(m);
              delete that.markers[message.ns + message.id];
          }
        }
        if(!updated) { // ADD
          var newMarker = new ROS3D.Marker({
            message : message,
            path : that.path,
            loader : that.loader,
            client : that,
            on_dblclick: that.on_dblclick,
            on_contextmenu: that.on_contextmenu
          });
          that.markers[message.ns + message.id] = new ROS3D.SceneNode({
            frameID : message.header.frame_id,
            tfClient : that.tfClient,
            object : newMarker
          });
          markerScene(newMarker).add(that.markers[message.ns + message.id]);
        }
      }
      else if(message.action === 1) { // "DEPRECATED"
        console.warn('Received marker message with deprecated action identifier "1"');
      }
      else if(message.action === 2) { // "DELETE"
        that.on_delete(message.ns);
        
        var m = that.markers[message.ns + message.id];
        if(m) {
            m.unsubscribeTf();
            markerScene(m.object).remove(m);
            delete that.markers[message.ns + message.id];
        }
      }
      else if(message.action === 3) { // "DELETE ALL"
        for (var m in that.markers){
          m.unsubscribeTf();
          markerScene(m.object).remove(m);
          that.on_delete(m.ns);
        }
        that.markers = {};
      }
      else {
        console.warn('Received marker message with unknown action identifier "'+message.action+'"');
      }
    });
    
    that.emit('change');
  });
};
ROS3D.MarkerArrayClient.prototype.__proto__ = EventEmitter2.prototype;
