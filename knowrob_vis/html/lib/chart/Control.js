function Control(options) {
  var that = this;

  var ros = options.ros;

  var sliderLow = options.sliderLow || "";
  var sliderHigh = options.sliderHigh;
  var initButton = options.initButton;

  var startButton = options.startButton;
  var stopButton = options.stopButton;

  var timeDisplay = options.timeDisplay;
  var startTime = 0;
  var endTime = 0;

  var elapsedTime;
  that.elapsedTime = 0;
  var commandTimer;

  this.init = function() {
    console.log("control init");
    var startEndQuery = "findall(Time, task_start(T, Time), _List), sort(_List, _Sorted), nth0(0, _Sorted, X), last(_Sorted, Y), string_concat('http://knowrob.org/kb/cram_log.owl#timepoint_', Firststring, X), string_concat('http://knowrob.org/kb/cram_log.owl#timepoint_', Laststring, Y), atom_number(Firststring, First), atom_number(Laststring, Last)";

    var prolog = new JsonProlog(ros, {raw: true});
    prolog.jsonQuery(startEndQuery, function(result){
      prolog.finishClient();

      startTime = result.First;
      endTime = result.Last;
      console.log(result);
      console.log(startTime);
      console.log(endTime);

      var range = endTime - startTime;

      that.update(0);

      document.getElementById(sliderHigh).value = 0;  
      document.getElementById(sliderHigh).max = range;
      document.getElementById(sliderHigh).onchange = function(x) {
        var time = x.explicitOriginalTarget.value;
        console.log(time);
        that.update(time);
        that.elapsedTime = parseInt(time);
        var minutes = Math.floor(that.elapsedTime / 60);
        var seconds = that.elapsedTime - minutes * 60;
        document.getElementById(timeDisplay).innerHTML= minutes.toString() + ":" + seconds.toString();
      };

    });

    var minutes = Math.floor(that.elapsedTime / 60);
    var seconds = that.elapsedTime - minutes * 60;
    document.getElementById(timeDisplay).innerHTML= minutes.toString() + ":" + seconds.toString();

  }


  this.update = function(timePoint) {

    var actualTime = parseInt(timePoint) + parseInt(startTime);

    // change these queries as needed
    var robotPoseAtTime = "E = " + actualTime.toString() + ", mng_robot_pose_at_time(pr2:'PR2Robot1','/map',E,Pose), add_robot_as_basic_semantic_instance(Pose, E, SemanticMapInstance), add_object_with_children(SemanticMapInstance), highlight_object(SemanticMapInstance)";

    var queryErrorsUntilNow = "Time = " + actualTime.toString() + ",Labels = ['ManipulationPoseUnreachable', 'ObjectNotFound', 'ManipulationFailed', 'LocationNotReached', 'ObjectLost', 'ManipulationPoseOccupied'],findall(N, (member(Error, Labels),findall(E, (failure_class(E, C), C=knowrob:Error, failure_attribute(E, knowrob:startTime, Y), string_concat('http://knowrob.org/kb/cram_log.owl#timepoint_', Etimestring, Y), atom_number(Etimestring, Etime), Etime =< Time), List), length(List, N)),Occurence),add_diagram(errorsuntilnow, 'errors until now', barchart, xlabel, ylabel, '210', '210', '12px', [[Labels,Occurence]])";

    var queryTaskTreeUntilNow = "Time = " + actualTime.toString() + ", tasktree('http://knowrob.org/kb/cram_log.owl#CRAMAction_6gCmeZ3S', List), flatten(List, F), include("+'\\'+"T^(task_start(T, Timestring1), string_concat('http://knowrob.org/kb/cram_log.owl#timepoint_', Timestring2, Timestring1), atom_number(Timestring2, Starttime), Starttime =< Time), F, Flat), once(maplist("+'\\'+"X^Node^(((subtask(P, X), Parent = P); Parent = X), ((returned_value(X, Value), Color = red); Color = green), Node = [[X, Parent, Color],[X, Parent, Color]]), Flat, Nodes)), add_diagram(tree, title, treechart, xlabel, ylabel, '960', '500', '12px', Nodes)"

    console.log("new step");

    // error statistic until now
    var prolog = new JsonProlog(ros, {raw: true});
    prolog.jsonQuery(queryErrorsUntilNow, function(result){
      console.log("update errors until now!");
    });

    prolog.finishClient();

    // task tree until now
    var prolog = new JsonProlog(ros, {raw: true});
    prolog.jsonQuery(queryTaskTreeUntilNow, function(result){
      console.log("update task tree until now!");
    });

    prolog.finishClient();

    // robot pose at timepoint
    var prolog = new JsonProlog(ros, {raw: true});
    prolog.jsonQuery(robotPoseAtTime, function(result){
      console.log("update robot pose!");
    });

    prolog.finishClient();

  }

  document.getElementById(initButton).onclick = function() {
    document.getElementById(initButton).disabled = true;
    that.init();
  };

  document.getElementById(startButton).onclick = function() {
    document.getElementById(startButton).disabled = true;
    that.commandTimer = setInterval(function(){
      console.log("foo");
      if (that.elapsedTime > (endTime - startTime)){
        document.getElementById(startButton).disabled = false;
        clearInterval(that.commandTimer);
      }
      that.elapsedTime += 10;
      console.log(that.elapsedTime);
      that.update(that.elapsedTime);
      document.getElementById(sliderHigh).value = that.elapsedTime;
      
      var minutes = Math.floor(that.elapsedTime / 60);
      var seconds = that.elapsedTime - minutes * 60;
      document.getElementById(timeDisplay).innerHTML= minutes.toString() + ":" + seconds.toString();
    },3000);
  };

  document.getElementById(stopButton).onclick = function() {
    document.getElementById(startButton).disabled = false;
    clearInterval(that.commandTimer);
  };

}
