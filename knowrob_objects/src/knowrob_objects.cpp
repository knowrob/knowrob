#include "knowrob_objects.h"

#include <stdlib.h>

#include <ros/ros.h>
#include <knowrob_objects/DirtyObject.h>

PREDICATE(mark_dirty_objects, 1) {
  PlTail tail(PL_A1);
  PlTerm e;
  knowrob_objects::DirtyObject msg;
  while(tail.next(e)) {
    msg.request.object_ids.push_back(std::string((char*)e));
  }
  if (ros::service::call("/object_state_publisher/mark_dirty_object", msg)) {
    return TRUE;
  }
  else {
    return FALSE;
  }
}
