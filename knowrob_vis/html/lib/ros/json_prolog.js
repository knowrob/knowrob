
function JsonProlog(ros, options){

  var that = this;
  this.raw = options.raw || false;
  this.finished = false;
  var ros = ros;
  
  this.makeid = function() {

    var text = "";
    var possible = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";

    for( var i=0; i < 8; i++ )
        text += possible.charAt(Math.floor(Math.random() * possible.length));

    return text;
  };

  var qid = that.makeid();


  this.jsonQuery = function(query, callback) {
      
      console.log("jsonQuery()");
      
      //var qid = this.makeid();

      var jsonPrologQueryClient = new ROSLIB.Service({
        ros : ros,
        name : '/json_prolog/simple_query',
        serviceType : 'json_prolog/PrologQuery'
      });

      
      // send query
      var request = new ROSLIB.ServiceRequest({
        mode : 0,  // 1->INCREMENTAL, 0->ALL
        id : qid,
        query : query
      });

//       console.log(request);
    
      jsonPrologQueryClient.callService(request, function(result) {

        // collect results
        var jsonPrologNextResultClient = new ROSLIB.Service({
          ros : ros,
          name : '/json_prolog/next_solution',
          serviceType : 'json_prolog/PrologNextSolution'
        });

//         console.log(result);

        if (result.ok == true) {

          that.nextQuery(callback);

        } else {
          callback(result.message);
          that.finishClient();
        }
      });
      
      console.log("~jsonQuery()");
  };

  this.nextQuery = function (callback) {
      
      console.log("nextQuery()");

    var jsonPrologNextResultClient = new ROSLIB.Service({
      ros : ros,
      name : '/json_prolog/next_solution',
      serviceType : 'json_prolog/PrologNextSolution'
    });

    var request2 = new ROSLIB.ServiceRequest({
      id : qid
    });

    jsonPrologNextResultClient.callService(request2, function(result) {

//       console.log(result);
//       console.log(result.solution);

      // TODO result.solution should be parsed
      if (result.status == 0 && result.solution == "") {
        callback("false.\n\n");
        that.finishClient();
      } else if (result.status == 3 && result.solution == "{}") {
        callback("true.\n\n");
        that.finishClient();
      } else if (result.status == 3 && result.solution != "{}") {
        var solution = JSON.parse(result.solution);

        function parseSolution (solution, level, ret) {
          var indent = "";
          for (var i = 0; i < level; i++){indent += " "}
          for (var key in solution) {
            if (solution.hasOwnProperty(key)) {
              if (solution[key] instanceof Array || solution[key] instanceof Object) {
                ret += indent + key + " = [\n";
                //console.log("array!");
                ret = parseSolution(solution[key], level + 1, ret);
                ret += indent + "]\n"
              } else {
                ret += indent + key + " = " + solution[key] + "\n";
              }
            }
          }
          return ret;
        }
        if (that.raw == true) {
          var ret = solution;
        } else {
          var ret = parseSolution(solution, 0, "");
        }

//         console.log(solution);
        callback(ret);
      } //else {
//         console.log("wtf?");
//       }
    });

      console.log("~nextQuery()");

  };

  this.finishClient = function () {
      console.log("finishClient()");
      
    var jsonPrologFinishClient = new ROSLIB.Service({
      ros : ros,
      name : '/json_prolog/finish',
      serviceType : 'json_prolog/PrologFinish'
    });

    request3 = new ROSLIB.ServiceRequest({
      id : qid
    });

    jsonPrologFinishClient.callService(request3, function(e) { });
    that.finished = true;
    
      console.log("~finishClient()");
  };

}
