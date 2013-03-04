/*
 * main.cpp
 *
 *  Created on: Feb 8, 2013
 *      Author: stefan
 */

#include <iostream>
#include <fstream>

#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>

using namespace std;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}

bool readModelFile(const string &path, PointCloud &pc) {
	cout << "Reading file: "<< path << endl;

	ifstream myfile(path.c_str());
	string line;
	int count = -1;
	int curr = 0;
	pc.clear();
	if (myfile.is_open()) {
		while (myfile.good()) {
			getline(myfile, line, '\n');
			if (line.length() == 0)
				continue;
			if (count == -1) {
				count = atoi(line.c_str());
				if (count == 0)
					return true;
				continue;
			}
			std::vector<std::string> vals = split(line, '\t');
			if (vals.size() != 6) {
				cout << "Invalid values count '" << vals.size() << "' for point " << curr
						<< ". Expected 'x\ty\tz\tnx\tny\tnz'" << endl;
				pc.clear();
				return false;
			}
			pc.push_back(
					Point(
							Vec3f(atof(vals[0].c_str()), atof(vals[1].c_str()),
									atof(vals[2].c_str())),
							Vec3f(atof(vals[3].c_str()), atof(vals[4].c_str()),
									atof(vals[5].c_str()))));
			curr++;
		}
		myfile.close();
	} else {
		cout << "Unable to open file: " << path << endl;
		return false;
	}

	if (curr!=count) {
		cout << "Number of points indicated in header doesn't match final number of points: " << count <<"!="<<curr << endl;
		pc.clear();
		return false;
	}

	Vec3f min(pc[0][0],pc[0][1],pc[0][2]);
	Vec3f max(pc[0][0],pc[0][1],pc[0][2]);
	for (unsigned int i=1; i<pc.size(); i++) {
		min = Vec3f(std::min(min[0], pc[i][0]),
				std::min(min[1], pc[i][1]),
				std::min(min[2], pc[i][2]));
		max = Vec3f(std::max(max[0], pc[i][0]),
				std::max(max[1], pc[i][1]),
				std::max(max[2], pc[i][2]));
	}
	pc.setBBox(min, max);

	return true;
}

int main(int argc, char **argv) {

	if (argc != 3) {
		cout << "usage: ransac source.txt destination.txt" << endl;
		exit(EXIT_FAILURE);
	}

	string sourceFile(argv[1]);
	string destFile(argv[2]);

	PointCloud pc;
	if (!readModelFile(sourceFile, pc))
		exit(EXIT_FAILURE);
	cout << "File successfully read. Number of points: "<< pc.size() << endl;

	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = .01f * pc.getScale(); // set distance threshold to .01f of bounding box width
		// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
		// NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 10; // this is the minimal numer of points required for a primitive
	ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object

	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());
	detector.Add(new SpherePrimitiveShapeConstructor());
	detector.Add(new CylinderPrimitiveShapeConstructor());
	detector.Add(new ConePrimitiveShapeConstructor());
	detector.Add(new TorusPrimitiveShapeConstructor());

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes

	cout << "Starting detector ..." << endl;
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
	// returns number of unassigned points
	// the array shapes is filled with pointers to the detected shapes
	// the second element per shapes gives the number of points assigned to that primitive (the support)
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )
	cout << "Detector finished. Remaining points: " << remaining << endl;
	cout << "Found shapes: " << shapes.size() << endl;

	for (unsigned int i=0; i<shapes.size(); i++) {
		MiscLib::RefCountPtr< PrimitiveShape > shape = shapes[i].first;
		int shapePoints = shapes[i].second;
		string descr;
		shape->Description(&descr);
		cout << shapePoints << " = " << descr << endl;
	}

}
