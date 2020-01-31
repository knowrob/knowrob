#include <rosprolog/rosprolog_kb/rosprolog_kb.h>

#include <ros/ros.h>

template <typename T>
bool getParam(char *key, PlTerm &term) {
	T value;
	if(rosprolog_kb::node().getParam(std::string(key),value)) {
		term = value;
		return TRUE;
	}
	else {
		return FALSE;
	}
};
    
template <typename T>
bool setParam(char *key, const T &value) {
	rosprolog_kb::node().setParam(std::string(key),value);
	return TRUE;
};

/*********************************/
/********** Parameters ***********/
/*********************************/

PREDICATE(ros_param_get_string, 2) {
	std::string value;
	if(rosprolog_kb::node().getParam(std::string((char*)PL_A1),value)) {
		PL_A2 = value.c_str();
		return TRUE;
	}
	else {
		return FALSE;
	}
}

PREDICATE(ros_param_set_string, 2) {
	std::string val((char*)PL_A2);
	return setParam<std::string>((char*)PL_A1,val);
}

PREDICATE(ros_param_get_bool, 2) {
	PlTerm out = PL_A2;
	return getParam<bool>((char*)PL_A1,out);
}

PREDICATE(ros_param_set_bool, 2) {
	return setParam<bool>((char*)PL_A1,(int)PL_A2);
}

PREDICATE(ros_param_get_double, 2) {
	PlTerm out = PL_A2;
	return getParam<double>((char*)PL_A1,out);
}

PREDICATE(ros_param_set_double, 2) {
	return setParam<double>((char*)PL_A1,(double)PL_A2);
}

PREDICATE(ros_param_get_int, 2) {
	PlTerm out = PL_A2;
	return getParam<int>((char*)PL_A1,out);
}

PREDICATE(ros_param_set_int, 2) {
	return setParam<int>((char*)PL_A1,(int)PL_A2);
}
