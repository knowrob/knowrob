//jni
#include <jni.h>
#include "servicecpp.h"

//ros
#include "ros/ros.h"


#include <iostream>
using namespace std;

JNIEXPORT jobject JNICALL Java_edu_tum_cs_ias_knowrob_comp_1cap_CapROSClient_getHeader(
		JNIEnv *env, jclass in_cls, jstring j_dest_addr, jstring j_dest_port) {

	//convert java strings to c++ char arrays
	const char *c_dest_addr = (env)->GetStringUTFChars(j_dest_addr, 0);
	const char *c_dest_port = (env)->GetStringUTFChars(j_dest_port, 0);

	cout << "service_uri" << endl;
	cout << c_dest_addr << endl;
	cout << c_dest_port << endl;

	int argc  = 0;
	char** argv = "";

	ros::init(argc,argv,"CapROSClientService");
	ros::NodeHandle n;


	jobject result;

	return result;
}
