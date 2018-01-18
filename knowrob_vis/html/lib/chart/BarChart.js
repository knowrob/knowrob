function BarChart (options) {
  options = options || {};
  var w = options.width - 100 || 200;
  var h = options.height - 100 || 200;
  var data = options.data || [];
  var where = options.where;
  var label = options.label;
  var fontsize = options.fontsize || "14px";

    //setup the svg
  var svg = d3.select(where).append("svg:svg")
    .attr("width", w+100)
    .attr("height", h+100);

  svg.append("svg:rect")
    .attr("width", "100%")
    .attr("height", "100%")
    //.attr("stroke", "#000")
    .attr("fill", "none");

  var vis = svg.append("svg:g")
    //.attr("id", "barchart")
    .attr("transform", "translate(50,50)");

  var color = d3.scale.category20();

  this.remove = function() {
    svg.remove();
  }

  this.update = function(data) {

    max = d3.max(data.value2, function(d) {return parseInt(d)});

    var sum = data.value2.reduce(function(a,b) { return parseInt(a) + parseInt(b) });

    //nice breakdown of d3 scales
    //http://www.jeromecukier.net/blog/2011/08/11/d3-scales-and-color/
    x = d3.scale.linear()
        .domain([0, max])
        .range([0, w]);

    y = d3.scale.ordinal()
        .domain(d3.range(data.value2.length))
        .rangeBands([0, h], .2);
    
    //a good written tutorial of d3 selections coming from protovis
    //http://www.jeromecukier.net/blog/2011/08/09/d3-adding-stuff-and-oh-understanding-selections/
    var bars = vis.selectAll("rect.bar")
        .data(data.value2);

    //update
    bars
        .attr("fill", function(d, i) { return color(i); });
        //.attr("stroke", "black")//#050");
        //.attr("stroke-width", 0.1);

    //enter
    bars.enter()
        .append("svg:rect")
        .attr("class", "bar")
        .attr("fill", function(d, i) { return color(i); });//"#0a0") // #800
        //.attr("stroke", "#050"); // #800


    //exit 
    bars.exit()
    .transition()
    .duration(300)
    .ease("exp")
        .attr("width", 0)
        .remove();


    bars
        .attr("stroke-width", 4)
    .transition()
    .duration(300)
    .ease("quad")
        .attr("width", x)
        .attr("height", y.rangeBand())
        .attr("transform", function(d,i) {
            return "translate(" + [0, y(i)] + ")"
        });


    var text = vis.selectAll("text.value")
      .data(data.value2)
      .attr("x", 5)//x)
      .attr("y", function(d,i){ return y(i) + y.rangeBand()/2; } );

    text
      .enter().append("text")
      .attr("class", "value")
      .attr("x", 5)//x)
      .attr("y", function(d,i){ return y(i) + y.rangeBand()/2; } )
      //.attr("dx", -5)
      .attr("dy", ".36em")
      .attr("text-anchor", "start")
      .style("font-size", fontsize);

    text
     .text(function(d,i) {return data.value1[i]});//function(d) { return d; });

    text.exit()
      .remove();

    var percent = vis.selectAll("text.percent")
      .data(data.value2)
      .attr("x", 0)//x)
      .attr("y", function(d,i){ return y(i) + y.rangeBand()/2; } );

    percent
      .enter().append("text")
      .attr("class", "percent")
      .attr("x", 0)//x)
      .attr("y", function(d,i){ return y(i) + y.rangeBand()/2; } )
      //.attr("dx", -5)
      .attr("dy", ".36em")
      .attr("text-anchor", "end")
      .style("font-size", fontsize);

    percent
     .text(function(d,i) {return (100*parseInt(data.value2[i])/sum).toFixed(1) + "%" });//function(d) { return d; });

    percent.exit()
      .remove();

    var total = vis.selectAll("text.total")
      .data([sum]);
      //.attr("x", 5)//x)
      //.attr("y", h+10);//function(d,i){ return y(i) + y.rangeBand()/2; } );

    total
      .enter().append("text")
      .attr("class", "total")
      .attr("x", 5)//x)
      .attr("y", h+40)
      //.attr("dx", -5)
      .attr("dy", 0)//".36em")
      .attr("text-anchor", "start")
      .style("font-size", fontsize);

    total
     .text(sum + " - " + label);//Total "+sum);//function(d) { return d; });

    total.exit()
      .remove();

  }

}
