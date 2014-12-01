//inspired by http://blog.stephenboak.com/2011/08/07/easy-as-a-pie.html

function DonutChart(options) {
  options = options || {};
  var w = options.width || 300;
  var h = options.height || 300;
  var r = options.radius || 90;//100
  var ir = options.innerRadius || 40;//45
  var textOffset = 14;
  var tweenDuration = 250;
  var where = options.where || "body";
  var data = options.data || [];
  var label = options.label || "units";
  var fontsize = options.fontsize || "14px";

  //OBJECTS TO BE POPULATED WITH DATA LATER
  var lines, valueLabels, nameLabels;
  var pieData = [];
  var oldPieData = [];
  var filteredPieData = [];

  //D3 helper function to populate pie slice parameters from array data
  var donut = d3.layout.pie().value(function(d){
    return parseInt(d);
  });

  //D3 helper function to create colors from an ordinal scale
  var color = d3.scale.category20();

  //D3 helper function to draw arcs, populates parameter "d" in path object
  var arc = d3.svg.arc()
    .startAngle(function(d){ return d.startAngle; })
    .endAngle(function(d){ return d.endAngle; })
    .innerRadius(ir)
    .outerRadius(r);


  // CREATE VIS & GROUPS
  // -------------------

  var vis = d3.select(where).append("svg:svg")
    .attr("width", w)
    .attr("height", h);

  //GROUP FOR ARCS/PATHS
  var arc_group = vis.append("svg:g")
    .attr("class", "arc")
    .attr("transform", "translate(" + (w/2) + "," + (h/2) + ")");

  //GROUP FOR LABELS
  var label_group = vis.append("svg:g")
    .attr("class", "label_group")
    .attr("transform", "translate(" + (w/2) + "," + (h/2) + ")");

  //GROUP FOR CENTER TEXT  
  var center_group = vis.append("svg:g")
    .attr("class", "center_group")
  .attr("transform", "translate(" + (w/2) + "," + (h/2) + ")");

  //PLACEHOLDER GRAY CIRCLE
  var paths = arc_group.append("svg:circle")
    .attr("fill", "#EFEFEF")
    .attr("r", r);

  // CENTER TEXT
  // -----------

  //WHITE CIRCLE BEHIND LABELS
  var whiteCircle = center_group.append("svg:circle")
    .attr("fill", "white")
    .attr("r", ir);

  // "TOTAL" LABEL
  /*var totalLabel = center_group.append("svg:text")
    .attr("class", "label")
    .attr("dy", -15)
    .attr("text-anchor", "middle") // text-align: right
    .text("TOTAL");*/

  //TOTAL TRAFFIC VALUE
  var totalValue = center_group.append("svg:text")
    .attr("class", "total")
    .attr("dy", 7)
    .attr("text-anchor", "middle") // text-align: right
    .style("font-size", fontsize)
    .text("Waiting...");

  //UNITS LABEL
  var totalUnits = center_group.append("svg:text")
    .attr("class", "units")
    .attr("dy", h/2-10)//120)//21)
    .attr("text-anchor", "middle") // text-align: right
    .style("font-size", fontsize)
    .text(label);

  // removes this chart
  this.remove = function() {
    vis.remove();
  }

  // update data
  // -----------

  this.update = function(data) {

    //console.log(data);

    oldPieData = filteredPieData;
    pieData = donut(data.value2);

    var totalElements = 0;
    filteredPieData = pieData.filter(filterData);
    function filterData(element, index, array) {
      element.name = data.value1[index];
      element.value = parseInt(data.value2[index]);
      totalElements += element.value;
      return (element.value > 0);
    }
    //console.log(filteredPieData);

    if(filteredPieData.length > 0 ) {//&& oldPieData.length > 0){

      //REMOVE PLACEHOLDER CIRCLE
      arc_group.selectAll("circle").remove();

      totalValue.text(function(){
        return totalElements;
      });

      //DRAW ARC PATHS
      paths = arc_group.selectAll("path").data(filteredPieData);
      paths.enter().append("svg:path")
        .attr("stroke", "white")
        .attr("stroke-width", 0.5)
        .attr("fill", function(d, i) { return color(i); })
        .transition()
          .duration(tweenDuration)
          .attrTween("d", pieTween);
      paths
        .transition()
          .duration(tweenDuration)
          .attrTween("d", pieTween);
      paths.exit()
        .transition()
          .duration(tweenDuration)
          .attrTween("d", removePieTween)
        .remove();

      //DRAW TICK MARK LINES FOR LABELS
      lines = label_group.selectAll("line").data(filteredPieData);
      lines.enter().append("svg:line")
        .attr("x1", 0)
        .attr("x2", 0)
        .attr("y1", -r-3)
        .attr("y2", -r-8)
        .attr("stroke", "gray")
        .attr("transform", function(d) {
          return "rotate(" + (d.startAngle+d.endAngle)/2 * (180/Math.PI) + ")";
        });
      lines.transition()
        .duration(tweenDuration)
        .attr("transform", function(d) {
          return "rotate(" + (d.startAngle+d.endAngle)/2 * (180/Math.PI) + ")";
        });
      lines.exit().remove();

      //DRAW LABELS WITH PERCENTAGE VALUES
      valueLabels = label_group.selectAll("text.value").data(filteredPieData)
        .attr("dy", function(d){
          if ((d.startAngle+d.endAngle)/2 > Math.PI/2 && (d.startAngle+d.endAngle)/2 < Math.PI*1.5 ) {
            return 5;
          } else {
            return -7;
          }
        })
        .attr("text-anchor", function(d){
          if ( (d.startAngle+d.endAngle)/2 < Math.PI ){
            return "beginning";
          } else {
            return "end";
          }
        })
        .style("font-size", fontsize)
        .text(function(d){
          var percentage = (d.value/totalElements)*100;
          return percentage.toFixed(1) + "%";
        });

      valueLabels.enter().append("svg:text")
        .attr("class", "value")
        .attr("transform", function(d) {
          return "translate(" + Math.cos(((d.startAngle+d.endAngle - Math.PI)/2)) * (r+textOffset) + "," + Math.sin((d.startAngle+d.endAngle - Math.PI)/2) * (r+textOffset) + ")";
        })
        .attr("dy", function(d){
          if ((d.startAngle+d.endAngle)/2 > Math.PI/2 && (d.startAngle+d.endAngle)/2 < Math.PI*1.5 ) {
            return 5;
          } else {
            return -7;
          }
        })
        .attr("text-anchor", function(d){
          if ( (d.startAngle+d.endAngle)/2 < Math.PI ){
            return "beginning";
          } else {
            return "end";
          }
        })
        .style("font-size", fontsize)
        .text(function(d){
          var percentage = (d.value/totalElements)*100;
          return percentage.toFixed(1) + "%";
        });

      valueLabels.transition().duration(tweenDuration).attrTween("transform", textTween);

      valueLabels.exit().remove();


      //DRAW LABELS WITH ENTITY NAMES
      nameLabels = label_group.selectAll("text.units").data(filteredPieData)
        .attr("dy", function(d){
          if ((d.startAngle+d.endAngle)/2 > Math.PI/2 && (d.startAngle+d.endAngle)/2 < Math.PI*1.5 ) {
            return 17;
          } else {
            return 5;
          }
        })
        .attr("text-anchor", function(d){
          if ((d.startAngle+d.endAngle)/2 < Math.PI ) {
            return "beginning";
          } else {
            return "end";
          }
        })
        .style("font-size", fontsize)
        .text(function(d){
          return d.name;
        });

      nameLabels.enter().append("svg:text")
        .attr("class", "units")
        .attr("transform", function(d) {
          return "translate(" + Math.cos(((d.startAngle+d.endAngle - Math.PI)/2)) * (r+textOffset) + "," + Math.sin((d.startAngle+d.endAngle - Math.PI)/2) * (r+textOffset) + ")";
        })
        .attr("dy", function(d){
          if ((d.startAngle+d.endAngle)/2 > Math.PI/2 && (d.startAngle+d.endAngle)/2 < Math.PI*1.5 ) {
            return 17;
          } else {
            return 5;
          }
        })
        .attr("text-anchor", function(d){
          if ((d.startAngle+d.endAngle)/2 < Math.PI ) {
            return "beginning";
          } else {
            return "end";
          }
        })
        .style("font-size", fontsize)
        .text(function(d){
          return d.name;
        });

      nameLabels.transition().duration(tweenDuration).attrTween("transform", textTween);

      nameLabels.exit().remove();
    }  
  }

// Interpolate the arcs in data space.
function pieTween(d, i) {
    var s0;
    var e0;
    if(oldPieData[i]){
      s0 = oldPieData[i].startAngle;
      e0 = oldPieData[i].endAngle;
    } else if (!(oldPieData[i]) && oldPieData[i-1]) {
      s0 = oldPieData[i-1].endAngle;
      e0 = oldPieData[i-1].endAngle;
    } else if(!(oldPieData[i-1]) && oldPieData.length > 0){
      s0 = oldPieData[oldPieData.length-1].endAngle;
      e0 = oldPieData[oldPieData.length-1].endAngle;
    } else {
      s0 = 0;
      e0 = 0;
    }
    var i = d3.interpolate({startAngle: s0, endAngle: e0}, {startAngle: d.startAngle, endAngle: d.endAngle});
    return function(t) {
      var b = i(t);
      return arc(b);
    };
  }

  function removePieTween(d, i) {
    s0 = 2 * Math.PI;
    e0 = 2 * Math.PI;
    var i = d3.interpolate({startAngle: d.startAngle, endAngle: d.endAngle}, {startAngle: s0, endAngle: e0});
    return function(t) {
      var b = i(t);
      return arc(b);
    };
  }

  function textTween(d, i) {
    var a;
    if(oldPieData[i]){
      a = (oldPieData[i].startAngle + oldPieData[i].endAngle - Math.PI)/2;
    } else if (!(oldPieData[i]) && oldPieData[i-1]) {
      a = (oldPieData[i-1].startAngle + oldPieData[i-1].endAngle - Math.PI)/2;
    } else if(!(oldPieData[i-1]) && oldPieData.length > 0) {
      a = (oldPieData[oldPieData.length-1].startAngle + oldPieData[oldPieData.length-1].endAngle - Math.PI)/2;
    } else {
      a = 0;
    }
    var b = (d.startAngle + d.endAngle - Math.PI)/2;
  
    var fn = d3.interpolateNumber(a, b);
    return function(t) {
      var val = fn(t);
      return "translate(" + Math.cos(val) * (r+textOffset) + "," + Math.sin(val) * (r+textOffset) + ")";
    };
  }

}
