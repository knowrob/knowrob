/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 */

/**
 * A Viewer can be used to render an interactive 3D scene to a HTML5 canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * divID - the ID of the div to place the viewer in
 *  * width - the initial width, in pixels, of the canvas
 *  * height - the initial height, in pixels, of the canvas
 *  * background (optional) - the color to render the background, like '#efefef'
 *  * antialias (optional) - if antialiasing should be used
 *  * intensity (optional) - the lighting intensity setting to use
 *  * cameraPosition (optional) - the starting position of the camera
 */
ROS3D.Viewer = function(options) {
  var that = this;
  options = options || {};
  var divID = options.divID;
  var width = options.width;
  var height = options.height;
  var background = options.background || '#111111';
  var antialias = options.antialias;
  var intensity = options.intensity || 0.66;
  var near = options.near || 0.01;
  var far = options.far || 1000;
  var on_window_dblclick = options.on_window_dblclick || function() {};
  var cameraPosition = options.cameraPose || {
    x : 3,
    y : 3,
    z : 3
  };
  var cameraZoomSpeed = options.cameraZoomSpeed || 0.5;

  // create the canvas to render to
  this.renderer = new THREE.WebGLRenderer({
    preserveDrawingBuffer: true,
    antialias : true
  });
  this.renderer.setClearColor(parseInt(background.replace('#', '0x'), 16), 1.0);
  this.renderer.sortObjects = false;
  this.renderer.setSize(width, height);
  this.renderer.autoClear = false;
  
  if(options.enableShadows) {
      this.renderer.shadowMapEnabled = true;
      this.renderer.shadowMapSoft = true;
      this.renderer.shadowMapType = THREE.PCFSoftShadowMap;
  }
  else {
      this.renderer.shadowMapEnabled = false;
  }
  
  // create the global scene
  this.scene = new THREE.Scene();
  // create the global scene for HUD
  this.sceneOrtho = new THREE.Scene();

  // create the global camera
  //this.camera = new THREE.PerspectiveCamera(40, width / height, near, far);
  this.camera = new THREE.PerspectiveCamera(81.4, width / height, near, far);
  this.camera.position.x = cameraPosition.x;
  this.camera.position.y = cameraPosition.y;
  this.camera.position.z = cameraPosition.z;
  // add controls to the camera
  this.cameraControls = new ROS3D.OrbitControls({
    scene : this.scene,
    camera : this.camera
  });
  this.cameraControls.userZoomSpeed = cameraZoomSpeed;
  this.camera.setViewOffset( 1920, 1080, 370, 164, 1155, 736);
  
  // create the global camera with orthogonal projection
  this.cameraOrtho = new THREE.OrthographicCamera( - width / 2, width / 2, height / 2, - height / 2, 1, 10 );
  this.cameraOrtho.position.z = 10;

  // lights
  this.scene.add(new THREE.AmbientLight(0x555555));
  /*
  this.directionalLight = new THREE.DirectionalLight(0x880000, intensity);
  that.directionalLight.position = new THREE.Vector3(-1, -1, 1);
  that.directionalLight.position.normalize();
  this.scene.add(this.directionalLight);
  */

  // propagates mouse events to three.js objects
  this.selectableObjects = new THREE.Object3D();
  this.scene.add(this.selectableObjects);
  var mouseHandler = new ROS3D.MouseHandler({
    renderer : this.renderer,
    camera : this.camera,
    rootObject : this.selectableObjects,
    fallbackTarget : this.cameraControls,
    on_window_dblclick : on_window_dblclick
  });

  // highlights the receiver of mouse events
  this.highlighter = new ROS3D.Highlighter({
    mouseHandler : mouseHandler
  });
  
  this.backgroundScene = new THREE.Scene();
  this.backgroundCamera = new THREE.Camera();
  this.backgroundScene.add(this.backgroundCamera);
  
  var renderEvent = {
      'camera': that.camera,
      'scene': that.scene
  };
  
  /**
   * Renders the associated scene to the viewer.
   */
  function draw() {
    // update the controls
    that.cameraControls.update();
    
    // notify listener about the draw call
    that.emit('render', renderEvent);

    // put light to the top-left of the camera
    //that.directionalLight.position = that.camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    //that.directionalLight.position.normalize();
    
    that.renderer.clear(true, true, true);
    that.renderer.render(that.backgroundScene, that.backgroundCamera);
    that.renderer.render(that.scene, that.camera);

    // render any mouseovers
    that.highlighter.renderHighlight(that.renderer, that.scene, that.camera);

    // draw the frame
    requestAnimationFrame(draw);
    
    // draw HUD
    that.renderer.render(that.sceneOrtho, that.cameraOrtho);
  }

  // add the renderer to the page
  // TODO(daniel): not using document.getElementById(divID) here due to frames
  document.getElementById(divID).appendChild(this.renderer.domElement);
  //divID.appendChild(this.renderer.domElement);

  // begin the animation
  draw();
};

/**
 * Add the given THREE Object3D to the global scene in the viewer.
 *
 * @param object - the THREE Object3D to add
 * @param selectable (optional) - if the object should be added to the selectable list
 */
ROS3D.Viewer.prototype.addObject = function(object, selectable) {
  if (selectable) {
    this.selectableObjects.add(object);
  } else {
    this.scene.add(object);
  }
};

/**
 * Resize 3D viewer
 *
 * @param width - new width value
 * @param height - new height value
 */
ROS3D.Viewer.prototype.resize = function(width, height) {
  this.camera.width = width;
  this.camera.height = height;
  this.camera.aspect = width / height;
  this.camera.updateProjectionMatrix();
  
  // update orthographic projection
  this.cameraOrtho.width = width;
  this.cameraOrtho.height = height;
  this.cameraOrtho.left = - width / 2;
  this.cameraOrtho.right = width / 2;
  this.cameraOrtho.top = height / 2;
  this.cameraOrtho.bottom = - height / 2;
  this.cameraOrtho.updateProjectionMatrix();
  
  this.renderer.setSize(width, height);
};

ROS3D.Viewer.prototype.__proto__ = EventEmitter2.prototype;
