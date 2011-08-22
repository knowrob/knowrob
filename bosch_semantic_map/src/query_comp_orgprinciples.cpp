#include <string>
#include <iostream>

#include <unistd.h> //for sleep()

#include <ros/ros.h>
#include <json_prolog/prolog.h>

using namespace std;
using namespace json_prolog;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "best_object_location");

  Prolog pl;
    cout << "1" << std::endl;
  //classifiers: best_location_maxMaxWup and best_location_dtree
  //best_location_maxMaxWup may return multiple solutions (if they
  //have equal similarities)
   string locationMilk = "";
  //query: get location as string
  {
    string object = "kitchenobjects:'CoffeeStarbucks'";
    string classifier = "best_location_maxMaxWup"; //best_location_dtree
   
    string query = classifier + "(" + object + ", L)";
    PrologQueryProxy bdgs = pl.query(query);
       cout << "2" << std::endl;
    for(PrologQueryProxy::iterator it=bdgs.begin();
	it != bdgs.end(); it++)
      {
cout << "3" << std::endl;
	PrologBindings bdg = *it;
	//cout << "Found solution: " << (bool)(it == bdgs.end()) << endl;
	cout << "Location = "<< bdg["L"].toString() << endl;
        locationMilk = (bdg["L"].toString());
      }
cout << "4" << std::endl;
  }
    
    
  //query and highlight in knowrob visualization
  {
cout << "5" << std::endl;
    //initialize visualization:
    pl.query("register_ros_package(mod_vis)");
cout << "6" << std::endl;
    pl.query("use_module(library('mod_vis'))");
cout << "7" << std::endl;
    pl.query("mod_vis:visualisation_canvas(C)");
cout << "8" << std::endl;
    //wait until it is initialized
    sleep(5);
    
    string object = "kitchenobjects:'CoffeeStarbucks'";
    string classifier = "best_location_maxMaxWup"; //best_location_dtree

    string query = "highlight_" + classifier + "(" + object + ", _)";
    cout << query << endl;
    pl.query(query);
 
    
    sleep(10);
    
    object = "kitchenobjects:'CoffeeStarbucks'";
    query = "highlight_" + classifier + "(" + object + ", _)";
    cout << query << endl;
    pl.query(query);
  }

    //display images
    {
        string query = "display_object_images_at_location('" + locationMilk + "','/usr/stud/schuster/mapping-private/shopping_demo')";
        cout << query << endl;
        pl.query(query);
    }
  
  return 0;
}
