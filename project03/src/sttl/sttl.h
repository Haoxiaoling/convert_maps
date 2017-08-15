#ifndef STTL
#define STTL


// #include <stdlib.h>
// #include <stdio.h>

#include <Eigen/Geometry>

#include <iomanip>
#include <iostream>

#include <string>
#include <list>
#include <stdint.h>

namespace sttl{


/*
 * A Transform between two frames
 * There might be multiple transforms for every frame - maybe even with the same reference frame.
 * Those are not specially handeled here - just more than one Transform will exist.
 */
class Transform{
public:

	struct Id{
		uint64_t id[2];
		Id(){
			id[0] = 0;
			id[1] = 0;
			// id[2] = 0;
			// id[3] = 0;
		}
		Id(uint64_t id0, uint64_t id1){
			id[0] = id0;
			id[1] = id1;
			// id[2] = id2;
			// id[3] = id3;
		}
		bool operator==(const Id& other) const{
			return id[0] == other.id[0] && id[1] == other.id[1];
		}

		std::string toString() const;
		bool fromString(const std::string& string);

	};

	std::string frameName;	//< the name of the frame we are describing
	std::string referenceFrameName;  //< in case the reference Frame cannot be found

	Eigen::Transform<double,3,Eigen::Affine> transform; //< the transfrom of this frame wrt the reference frame
	bool hasCovariance; //< only true if the covariance matrix below has proper values
	Eigen::Matrix<double, 7, 7> covariance; //< the 7x7 covariance matrix of translation and rotation as quaternion

	std::string byUser;	//< The user responsible for generating this transform
	std::string time;	//< the time this transform was generated

	std::string method;	//< Name of the method used to generate this transform - use the constants below if possible
	std::string program; //< Name of the program used to generate this transform (e.g. "cloudcompare")
	std::string details; //< Any additional data that this method/ program wants to save

	Id id; //< special 256bit (hopefully) unique id generated once by generateId()

	std::list<Id> history; //< list of other transforms between these frames that influenced this result.

	Transform():hasCovariance(false){}

	bool generateId();

};

static const std::string METHOD_MANUAL = "Manual";
static const std::string METHOD_POINT_PAIRS = "Point Pairs";
static const std::string METHOD_ICP = "ICP";
static const std::string METHOD_G2O = "g2o";

class Frame{
public:
	std::string name;
	std::string dataNames;

	std::list<std::list<sttl::Transform>::iterator> children;
	std::list<std::list<sttl::Transform>::iterator> parents;

	Eigen::Transform<double,3,Eigen::Affine> rootTransform; //< the transfrom of this frame wrt root

	bool visited;
};


bool generateFrames(std::list<sttl::Transform> & transforms, std::list<Frame> &frames);

std::list<sttl::Frame>::iterator findFrame(std::list<sttl::Frame> &frames, const std::string & frame);
std::list<sttl::Frame>::iterator findFrameForData(std::list<sttl::Frame> &frames, const std::string & dataName);

bool saveList(const std::list<sttl::Transform> & transforms, const std::string &file);
bool loadList(std::list<sttl::Transform> & transforms, const std::string &file);



}

#endif
