

  function format_designator(desig, pre, parent, level) {

    if(desig.length==0)
      return pre;

    post = pre;
    d = desig.shift()

    // context ended, go one level up
    if(d["parent"] < parent)
      level--;

    if(d["key"].substring(0, 1) != '_') {

      if(d["type"]==0) { // string
        post += "<div class='desig div"+level+"'>\n"
        post += d["key"];
        post += " = " + d["value_string"];
        post += "</div>\n"
        post = format_designator(desig, post, d["parent"], level);

      } if(d["type"]==1) { // float
        post += "<div class='desig div"+level+"'>\n"
        post += d["key"];
        post += " = " + d["value_float"];
        post += "</div>\n"
        post = format_designator(desig, post, d["parent"], level);

      } if(d["type"]==5) { // Pose
        post += "<div class='desig div"+level+"'>\n"
        post += d["key"];
        post += printPose(d["value_pose"], level);
        post += "</div>\n"
        post = format_designator(desig, post, d["parent"], level);

      } if(d["type"]==4) { // PoseStamped
        post += "<div class='desig div"+level+"'>\n"
        post += d["key"];
        post += printPoseStamped(d["value_posestamped"], level);
        post += "</div>\n"
        post = format_designator(desig, post, d["parent"], level);

      } if(d["type"]==3) {
        post += "<div class='desig div"+level+"'>\n"
        post += d["key"];
        post = format_designator(desig, post, d["parent"], level+1);
        post += "</div>\n"

      } if(d["type"]==6 || d["type"]==7 || d["type"]==8 ) {
        post += "<div class='desig div"+level+"'>\n"
        post += d["key"];
        post = format_designator(desig, post, d["parent"], level+1);
        post += "</div>\n"

      }
    }
    return post;
  };

  function printPoseStamped(poseStampedMsg, level) {
    post = "";
    post += "<div class='desig div"+(level+1)+"'>";
    post += "header<br/>\n";
      post += "<div class='desig div"+(level+2)+"'>";
      post += "seq = " + poseStampedMsg["header"]["seq"] + "<br/>\n";
      post += "stamp = " + poseStampedMsg["header"]["stamp"]["secs"] + "<br/>\n";
      post += "frame_id = " + poseStampedMsg["header"]["frame_id"] + "<br/>\n";
      post += "</div>\n";
    post += "pose<br/>\n";
      post += printPose(poseStampedMsg["pose"], level+1);
    post += "</div>\n";
    return post;
  }

  function printPose(poseMsg, level) {
    post = "";
    post += "<div class='desig div"+(level+1)+"'>";
    post += "position<br/>\n";
      post += "<div class='div"+(level+2)+"'>";
      post += "x = " + poseMsg["position"]["x"] + "<br/>\n";
      post += "y = " + poseMsg["position"]["y"] + "<br/>\n";
      post += "z = " + poseMsg["position"]["z"] + "<br/>\n";
      post += "</div>\n";
    post += "orientation<br/>\n";
      post += "<div class='desig div"+(level+2)+"'>";
      post += "x = " + poseMsg["orientation"]["x"] + "<br/>\n";
      post += "y = " + poseMsg["orientation"]["y"] + "<br/>\n";
      post += "z = " + poseMsg["orientation"]["z"] + "<br/>\n";
      post += "w = " + poseMsg["orientation"]["w"] + "<br/>\n";
      post += "</div>\n";
    post += "</div>\n";
    return post;
  };
