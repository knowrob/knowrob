/*
 * UrdfImporter
 * This small program converts an urdf file into a SRDL ontology
 * Author: Tobias Roehm, Jun 2010
 */

#include <urdf/model.h>
#include <iostream>
#include <fstream>
using namespace std;

// name definitions for owl uris, classes and properties
#define INTENDATION_STEP 4 // how much blanks are used as one intendation step in XML file
#define HTTP "http:/"
#define SRDL_COMP_URI "/ias.cs.tum.edu/kb/SRDL_component.owl"
#define SRDL_COMP "SRDL_component"
#define HAS_SUCCEEDING_LINK_NAME "hasSucceedingLink"
#define HAS_SUCCEEDING_JOINT_NAME "hasSucceedingJoint"
#define HAS_SUCCESSOR_NAME "hasSuccessorInKinematicChain"
#define LINK_NAME "UrdfLink"
#define JOINT_NAME "UrdfJoint"
#define REVOLUTE_JOINT_NAME "RevoluteUrdfJoint"
#define CONTINUOUS_JOINT_NAME "ContinuousUrdfJoint"
#define PRISMATIC_JOINT_NAME "PrismaticUrdfJoint" 
#define FIXED_JOINT_NAME "FixedUrdfJoint" 
#define FLOATING_JOINT_NAME "FloatingUrdfJoint"
#define PLANAR_JOINT_NAME "PlanarUrdfJoint"
#define HAS_URDF_NAME "hasUrdfName"


class urdfImporter{

    // Shared variables
    string ontologyFilename;
    string ontologyUri;
    string srdlUri;
    string urdfFilename;
    urdf::Model model;
    ofstream out;
    string namePrefix;  // can be used if a prefix should be put in front of every instance name

    public:

    // Main routine of conversion
    int controllMethod(int argc, char** argv) {
     
        // Initialization
        //ROS_INFO("Initializing ...");
        ros::init(argc, argv, "urdfConverter");
 
        // Establishing of URDF model
        ROS_INFO("Parsing URDF file ...");
        urdfFilename = argv[1];
        if (!model.initFile(urdfFilename)){
            ROS_ERROR("Failed to parse urdf file");
            ROS_ERROR("Please make sure to provide a valid urdf file");
            ROS_ERROR("Urdf files can be checked with the ROS command 'rosrun urdf check_urdf my_urdf.xml'");
            return -1;
        }
        ROS_INFO("Successfully parsed urdf file");

        // Creating file
        //ROS_INFO("Creating owl file ...");
        cout << "Please enter file name of ontology file:";
        getline(cin, ontologyFilename);
        //ontologyFilename = "ont.owl"; 
        out.open(ontologyFilename.c_str());

        // Write header
        //ROS_INFO("Writing file header ...");
        writeHeader();

        // Write model 
        ROS_INFO("Writing urdf model ...");
        writeModel();

        // Write footer 
        //ROS_INFO("Writing file footer ...");
        writeFooter();

        // Cleanup
        out.close();
        ROS_INFO("Ontology file has been written.");
        return 0;
    }

    /* Write xml and owl header for ontology */
    void writeHeader()  {
      
        // Read needed information from user 
        // Read Ontology URL
        cout << "Please enter ontology url (will be used in exactly the entered way):";
        getline(cin, ontologyUri); 
        //ontologyUri = "http://ont.owl";
        // Read Path to SRDL ontology
        srdlUri = "file:///usr/stud/roehm/ros/tumros-internal/stacks/knowrob/mod_srdl/owl/SRDL.owl"; 
        //cout << "Please enter path of SRDL ontology (will be used in exactly the entered way):";
        //getline(cin, srdlUri); 

       // write
       out << "<?xml version=\"1.0\"?>\n\n";
       out << "<!DOCTYPE rdf:RDF [ \n" << 
              "  <!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n" <<
              "  <!ENTITY " << SRDL_COMP << " \"" << HTTP << SRDL_COMP_URI << "#\" >\n" <<
              "  <!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" >\n" <<
              "  <!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\">\n" << 
              "]>\n\n";
       out << "<rdf:RDF xmlns=\"" << ontologyUri << "#\"\n" << 
              "  xml:base=\"" << ontologyUri << "\"\n" <<
              "  xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\"\n" <<
              "  xmlns:owl=\"http://www.w3.org/2002/07/owl#\"\n" <<
              "  xmlns:rdfs=\"http://www.w3.org/2000/01/rdf-schema#\"\n" <<
              "  xmlns:" << SRDL_COMP << "=\"" << HTTP << SRDL_COMP_URI << "#\">\n\n";
       out << "  <owl:Ontology rdf:about=\"\"> \n" <<
              "    <owl:imports rdf:resource=\"" << srdlUri << "\"/>\n" <<
              "  </owl:Ontology>\n\n";
    }

    /* Write a line, intended by intention blanks, in stream */
    void writeToStreamIntended(string line, int intendation) {

        // intention
        for (int i=0; i<intendation; i++) {
            out << " "; 
        }

        // write text
        out << line;
        out << "\n";
    }

    /* Iterate over the urdf model and write each of its components as a new individual in ontology */
    void writeModel() {

        // ask for a name prefix
        cout << "Please enter a prefix that will be put in front of all instance names:";
        getline(cin, namePrefix); 
        //namePrefix = "PR2_";
 
        // write general stuff
        out << "    <!--\n" <<
               "    ///////////////////////////////////////////////////////////////////////////////////////\n" <<
               "    //\n" <<
               "    // Datatype Properties\n" <<
               "    //\n" << 
               "    ///////////////////////////////////////////////////////////////////////////////////////\n" <<
               "    -->\n\n";

        out << "    <!-- " << HTTP << SRDL_COMP_URI << "#" << HAS_URDF_NAME << " -->\n" <<
               "    <owl:DatatypeProperty rdf:about=\"&" << SRDL_COMP << ";" << HAS_URDF_NAME << "\"/>\n\n";


        out << "    <!--\n" <<
               "    ///////////////////////////////////////////////////////////////////////////////////////\n" <<
               "    //\n" <<
               "    // Object Properties\n" <<
               "    //\n" << 
               "    ///////////////////////////////////////////////////////////////////////////////////////\n" <<
               "    -->\n\n";

        out << "    <!-- " << HTTP << SRDL_COMP_URI << "#" << HAS_SUCCESSOR_NAME << " -->\n" <<
               "    <owl:ObjectProperty rdf:about=\"&" << SRDL_COMP << ";" << HAS_SUCCESSOR_NAME << "\"/>\n\n";

        out << "    <!-- " << HTTP << SRDL_COMP_URI << "#" << HAS_SUCCEEDING_JOINT_NAME << " -->\n" <<
               "    <owl:ObjectProperty rdf:about=\"&" << SRDL_COMP << ";" << HAS_SUCCEEDING_JOINT_NAME << "\"/>\n\n";

        out << "    <!-- " << HTTP << SRDL_COMP_URI << "#" << HAS_SUCCEEDING_LINK_NAME << " -->\n" <<
               "    <owl:ObjectProperty rdf:about=\"&" << SRDL_COMP << ";" << HAS_SUCCEEDING_LINK_NAME << "\"/>\n\n";

        out << "    <!--\n" <<
               "    ///////////////////////////////////////////////////////////////////////////////////////\n" <<
               "    //\n" <<
               "    // Individuals\n" <<
               "    //\n" << 
               "    ///////////////////////////////////////////////////////////////////////////////////////\n" <<
               "    -->\n\n\n";

        // Links 
        map<string, boost::shared_ptr<urdf::Link> > linkMap = model.links_;
        map<string, boost::shared_ptr<urdf::Link> >::iterator it_link; 
        string linkKey, linkName;
        boost::shared_ptr<urdf::Link> linkPtr; 
        vector<boost::shared_ptr<urdf::Joint> > childJoints; 
        boost::shared_ptr<urdf::Joint> jointPtr;
        string jointName;
        unsigned int j; 
        for(it_link = linkMap.begin(); it_link != linkMap.end(); it_link++) {
            // get relevant link properties 
            linkKey = (*it_link).first;
            linkName = namePrefix + linkKey; 
            linkPtr = (*it_link).second;
            childJoints = linkPtr->child_joints;

            // write to XML file
            //<!-- http://www.semanticweb.org/ontologies/2010/5/21/Ontology1277133023832.owl#Link1 -->
            out << "    <!-- " << ontologyUri << "#" << linkName << " -->\n";
            //<owl:Thing rdf:about="#Link1">
            out << "    <owl:Thing rdf:about=\"#" << linkName << "\">\n";
            //<rdf:type rdf:resource="&SRDL_component;UrdfLink"/>
            out << "        <rdf:type rdf:resource=\"&" << SRDL_COMP << ";" << LINK_NAME << "\"/>\n";
            // <SRDL_component:hasUrdfName>Dummy_Name</SRDL_component:hasUrdfName>
            out << "        <"  << SRDL_COMP << ":" << HAS_URDF_NAME << ">" << linkKey << "</" << SRDL_COMP << ":" << HAS_URDF_NAME << ">\n" ;
            
            // Iterate over childJoint vector
            for(j=0; j < childJoints.size(); j++) {
                jointPtr = childJoints[j]; 
                jointName = namePrefix + jointPtr->name;

                //<SRDL_component:hasSucceedingJoint rdf:resource="#Joint12"/>
                out << "        <" << SRDL_COMP << ":" << HAS_SUCCEEDING_JOINT_NAME << " rdf:resource=\"#" << jointName << "\"/>\n";
                out << "        <" << SRDL_COMP << ":" << HAS_SUCCESSOR_NAME << " rdf:resource=\"#" << jointName << "\"/>\n";
            } 

            //</owl:Thing>
            out << "    </owl:Thing>\n\n";
        }

        // Joints
        map<string, boost::shared_ptr<urdf::Joint> > jointMap = model.joints_;
        map<string, boost::shared_ptr<urdf::Joint> >::iterator it_joint; 
        string jointKey;
        string childLinkName; 
        int jointType; 
        string jointTypeName;
        for(it_joint = jointMap.begin(); it_joint != jointMap.end(); it_joint++) {
            // get relevant joint properties 
            jointKey = (*it_joint).first;
            jointName = namePrefix + jointKey; 
            jointPtr = (*it_joint).second;
            childLinkName = namePrefix + jointPtr->child_link_name;
            jointType = jointPtr->type;

            // getting correct joint type
            switch (jointType)
            {
              case 1: // REVOLUTE
                 jointTypeName = REVOLUTE_JOINT_NAME;
                 break;
              case 2: // CONTINUOUS 
                 jointTypeName = CONTINUOUS_JOINT_NAME;
                 break;
              case 3: // PRISMATIC 
                 jointTypeName = PRISMATIC_JOINT_NAME;
                 break;
              case 4: // FLOATING 
                 jointTypeName = FLOATING_JOINT_NAME;
                 break;
              case 5: // PLANAR 
                 jointTypeName = PLANAR_JOINT_NAME;
                 break;
              case 6: // FIXED 
                 jointTypeName = FIXED_JOINT_NAME;
                 break;
              default:
                 jointTypeName = JOINT_NAME;
            }

            // write to XML file
            //<!-- http://www.semanticweb.org/ontologies/2010/5/21/Ontology1277133023832.owl#JointBase1 -->
            out << "    <!-- " << ontologyUri << "#" << jointName << " -->\n";
            //<owl:Thing rdf:about="#JointBase1">
            out << "    <owl:Thing rdf:about=\"#" << jointName << "\">\n";
            //<rdf:type rdf:resource="&SRDL_component;UrdfJoint"/>
            out << "        <rdf:type rdf:resource=\"&" << SRDL_COMP << ";" << jointTypeName << "\"/>\n";
            // <SRDL_component:hasUrdfName>Dummy_Name</SRDL_component:hasUrdfName>
            out << "        <"  << SRDL_COMP << ":" << HAS_URDF_NAME << ">" << jointKey << "</" << SRDL_COMP << ":" << HAS_URDF_NAME << ">\n" ;
            //<SRDL_component:hasSucceedingLink rdf:resource="#Link1"/>
            out << "        <" << SRDL_COMP << ":" << HAS_SUCCEEDING_LINK_NAME << " rdf:resource=\"#" << childLinkName << "\"/>\n";
            out << "        <" << SRDL_COMP << ":" << HAS_SUCCESSOR_NAME << " rdf:resource=\"#" << childLinkName << "\"/>\n";
            //</owl:Thing>
            out << "    </owl:Thing>\n\n";
        } 
    }

    void printAllLinks(urdf::Model &model) {
        cout << "Iterating over links\n";
        map<string, boost::shared_ptr<urdf::Link> > linkMap = model.links_;
        map<string, boost::shared_ptr<urdf::Link> >::iterator it; 
        for (it = linkMap.begin(); it != linkMap.end(); it++) {
            cout << (*it).first << " => " << (*it).second << endl;
        }
    } 


    /* Write footer of xml/ owl file */
    void writeFooter(){
        out << "</rdf:RDF>"; 
    }

};  // end of class UrdfImporter

/* Main routine of urdf converter */
int main(int argc, char** argv){

    // Check arguments
    if (argc != 2){
        ROS_ERROR("Need a urdf file as argument");
        return -1;
    }

    // Handover control to controller object
    int feedback;
    urdfImporter c;
    feedback = c.controllMethod(argc, argv);  
    return feedback;
}

