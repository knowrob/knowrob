
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
  string q = "rdfs_individual_of(Fridge, 'http://knowrob.org/kb/knowrob.owl#Refrigerator'),";
         q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#orientation',Fridge,Pose),";
         q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m03',Pose,_X), strip_literal_type(_X, X)";

  PrologQueryProxy bdgs = pl.query(q);
  for(PrologQueryProxy::iterator it=bdgs.begin(); it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    cout << "Fridge = "<< bdg["Fridge"] << endl;
    cout << "Pose = "<< bdg["Pose"] << endl;
    cout << "X = "<< bdg["X"] << endl;
  }

  cout << endl<< endl<< endl;


  //pose of the handle
  q = "rdfs_individual_of(Fridge, 'http://knowrob.org/kb/knowrob.owl#Refrigerator'),";
  q+= "rdf_has(Fridge, knowrob:properPhysicalParts, Door),";
  q+= "rdf_has(Door, knowrob:properPhysicalParts, Handle),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#orientation',Handle,Pose),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m03',Pose,_X), strip_literal_type(_X, X)";

  bdgs = pl.query(q);

  for(PrologQueryProxy::iterator it=bdgs.begin(); it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    cout << "Fridge = "<< bdg["Fridge"] << endl;
    cout << "Handle = "<< bdg["Handle"] << endl;
    cout << "Pose = "<< bdg["Pose"] << endl;
    cout << "X = "<< bdg["X"] << endl;
  }

  cout << endl<< endl<< endl;

  q = "rdfs_individual_of(Fridge, 'http://knowrob.org/kb/knowrob.owl#Refrigerator'),";
  q+= "rdf_has(Fridge, knowrob:properPhysicalParts, Door),";
  q+= "rdf_has(Door, knowrob:properPhysicalParts, Handle),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#orientation',Handle,Pose),";

  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m00',Pose,_M00), strip_literal_type(_M00, M00),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m01',Pose,_M01), strip_literal_type(_M01, M01),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m02',Pose,_M02), strip_literal_type(_M02, M02),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m03',Pose,_M03), strip_literal_type(_M03, M03),";

  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m10',Pose,_M10), strip_literal_type(_M10, M10),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m11',Pose,_M11), strip_literal_type(_M11, M11),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m12',Pose,_M12), strip_literal_type(_M12, M12),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m13',Pose,_M13), strip_literal_type(_M13, M13),";

  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m20',Pose,_M20), strip_literal_type(_M20, M20),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m21',Pose,_M21), strip_literal_type(_M21, M21),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m22',Pose,_M22), strip_literal_type(_M22, M22),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m23',Pose,_M23), strip_literal_type(_M23, M23),";

  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m30',Pose,_M30), strip_literal_type(_M30, M30),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m31',Pose,_M31), strip_literal_type(_M31, M31),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m32',Pose,_M32), strip_literal_type(_M32, M32),";
  q+= "rdf_triple('http://knowrob.org/kb/knowrob.owl#m33',Pose,_M33), strip_literal_type(_M33, M33)";

  bdgs = pl.query(q);

  for(PrologQueryProxy::iterator it=bdgs.begin(); it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    cout << "Pose = "<< bdg["Pose"] << endl;
    cout << "["<< bdg["M00"] << "\t" << bdg["M01"] << "\t" << bdg["M02"] << "\t" << bdg["M03"] << "]"<< endl;
    cout << "["<< bdg["M10"] << "\t" << bdg["M11"] << "\t" << bdg["M12"] << "\t" << bdg["M13"] << "]"<< endl;
    cout << "["<< bdg["M20"] << "\t" << bdg["M21"] << "\t" << bdg["M22"] << "\t" << bdg["M23"] << "]"<< endl;
    cout << "["<< bdg["M30"] << "\t" << bdg["M31"] << "\t" << bdg["M32"] << "\t" << bdg["M33"] << "]"<< endl;
  }
  
  return 0;
}
