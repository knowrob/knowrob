
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>

using namespace std;
using namespace json_prolog;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_json_prolog");

  Prolog pl;

  PrologQueryProxy bdgs = pl.query("member(A, [1, 2, 3, 4]), B = ['x', A], C = foo(bar, A, B)");

  for(PrologQueryProxy::iterator it=bdgs.begin();
      it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    cout << "Found solution: " << (bool)(it == bdgs.end()) << endl;
    cout << "A = "<< bdg["A"] << endl;
    cout << "B = " << bdg["B"] << endl;
    cout << "C = " << bdg["C"] << endl;
  }
  return 0;
}
