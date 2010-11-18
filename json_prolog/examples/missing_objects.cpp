
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>

using namespace std;
using namespace json_prolog;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fridge_handle");

  Prolog pl;


  // pose of the fridge itself
  string q = "comp_missingObjectTypes(knowrob:'KitchenTable0', PerceivedObj, MissingObj, MissingTypes)";

  PrologQueryProxy bdgs = pl.query(q);
  for(PrologQueryProxy::iterator it=bdgs.begin(); it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    cout << "PerceivedObj = "<< bdg["PerceivedObj"] << endl;
    cout << "MissingObj = "<< bdg["MissingObj"] << endl;
    cout << "MissingTypes = "<< bdg["MissingTypes"] << endl;
  }

  return 0;
}
