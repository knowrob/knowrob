
function DataVisClient(options) {
  var ros = options.ros;
  var containerId = options.containerId;
  var topic = options.topic;
  //var width = options.width || 300;
  //var height = options.height || 300;

  var chartHandle = [];

  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'data_vis_msgs/DataVis'
  });


  rosTopic.subscribe(function(message) {
    if (chartHandle.findIndex(function (element, index, array) {
          if(element.id == message.id) {return true} else {return false}
        }, this) == -1 && message.values[0].value2.length != 0) {

      var options = {
        data: message.values[0],
        where: containerId,
        label: message.title,
        width: message.width,
        height: message.height,
        radius: (message.height-120)/2,//height*3/10,
        innerRadius: (message.height-120)/2*4/9,
        fontsize: message.fontsize//"14px"
      };

      // Pie chart
      if (message.type == 0) {
        chartHandle.push({
          id: message.id,
          handle: new DonutChart(options)
        });

        chartHandle.myFind(function (element, index, array) {
            if(element.id == message.id) {return true} else {return false}
          }, this)
          .handle.update(message.values[0]);

      // Bar chart
      } else if (message.type == 1) {

        chartHandle.push({
          id: message.id,
          handle: new BarChart(options)
        });

        chartHandle.find(function (element, index, array) {
            if(element.id == message.id) {return true} else {return false}
          }, this)
          .handle.update(message.values[0]);

      // Tree diagram
      } else if (message.type == 2) {
        //console.log("new tree node");
        chartHandle.push({
          id: message.id,
          handle: new TreeDiagram(options)
        });
        
        chartHandle.find(function (element, index, array) {
          if(element.id == message.id) {return true} else {return false}
        }, this)
        .handle.update(message.values);

      // Timeline (Gantt style)
      } else if (message.type == 3) {

        chartHandle.push({
          id: message.id,
          handle: new Timeline(options)
        });
        
        chartHandle.find(function (element, index, array) {
          if(element.id == message.id) {return true} else {return false}
        }, this)
        .handle.update(message.values);
      }

    } else if (message.values.length == 0) {//[0].value2.length == 0) {
      chartHandle.find(function (element, index, array) {
          if(element.id == message.id) {return true} else {return false}
        }, this)
        .handle.remove();
      
      chartHandle.splice(chartHandle.findIndex(function (element, index, array) {
          if(element.id == message.id) {return true} else {return false}
        }, this), 1);

    } else {
      if (message.type == 2) {
        chartHandle.find(function (element, index, array) {
            if(element.id == message.id) {return true} else {return false}
          }, this)
          .handle.update(message.values);
      } else {
        chartHandle.find(function (element, index, array) {
            if(element.id == message.id) {return true} else {return false}
          }, this)
          .handle.update(message.values[0]);
      }
    }
    //console.log(chartHandle);
  });
}
