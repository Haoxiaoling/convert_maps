#include "sttl.h"

#include <iostream>
#include <fstream>

#include <json/json.h>
#include <json/value.h>

#include <chrono>
#include <functional>

#include <cstdlib>


using namespace std;

std::string sttl::Transform::Id::toString() const{
	std::stringstream str;
	str.fill('0');
	str.width(16);
	str<<std::hex<<id[0];
	str.fill('0');
	str.width(16);
	str<<std::hex<<id[1];
	return str.str();
}
bool sttl::Transform::Id::fromString(const std::string& string){
	for(int i=0; i<2; ++i){
		std::stringstream str(string.substr(i*16,16));
		str>>std::hex>>id[i];
	}
	return true;
}



bool readJson(sttl::Transform & transform, Json::Value & node){
	transform.frameName = node["name"].asString();
	transform.referenceFrameName = node["referenceFrame"].asString();
	transform.id.fromString(node["id"].asString());

	transform.byUser = node["user"].asString();
	transform.method = node["method"].asString();
	transform.program = node["program"].asString();
	transform.details = node["details"].asString();
	transform.time = node["time"].asString();

	for(int i=0; i<node["history"].size(); ++i){
		sttl::Transform::Id id;
		if(id.fromString(node["history"][i].asString())){
			transform.history.push_back(id);
		}	
	}

	if(node["transform"].size()!=4){
		std::cerr<<"transform does not have 4 elements!"<<std::endl;
		return false;
	}
	if(node["transform"][0].size()!=4){
		std::cerr<<"transform does not have 4x4 elements!"<<std::endl;
		return false;
	}

	for(int i=0; i<4; ++i){
		for(int j=0; j<4; ++j){
			transform.transform(i, j) = node["transform"][i][j].asDouble();
		}
	}

	transform.hasCovariance = false;
	if(node.isMember("covariance")){
		transform.hasCovariance = true;
		if(node["covariance"].size()!=7){
			std::cerr<<"covariance does not have 7 elements!"<<std::endl;
			return false;
		}
		if(node["covariance"][0].size()!=7){
			std::cerr<<"covariance does not have 7x7 elements!"<<std::endl;
			return false;
		}
		for(int i=0; i<7; ++i){
			for(int j=0; j<7; ++j){
				transform.covariance(i, j) = node["covariance"][i][j].asDouble();
			}
		}
	}

	return true;
}

bool fillJson(const sttl::Transform & transform, Json::Value & node){
	node["name"] = transform.frameName;


	Json::Value trans(Json::arrayValue);
	trans.resize(4);
		for(int i=0; i<4; ++i){
			trans[i]=Json::Value(Json::arrayValue);
			trans[i].resize(4);
			for(int k=0; k<4; ++k){
				trans[i][k] = transform.transform(i, k);
			}
		}
	node["transform"] = trans;



	if(transform.hasCovariance){
	Json::Value cov(Json::arrayValue);
	cov.resize(7);
		for(int i=0; i<7; ++i){
			cov[i]=Json::Value(Json::arrayValue);
			cov[i].resize(7);
			for(int k=0; k<7; ++k){
				cov[i][k] = transform.covariance(i, k);
			}
		}
		node["covariance"] = cov;
	}

	node["referenceFrame"] = transform.referenceFrameName;
	node["id"] = transform.id.toString();

	node["user"] = transform.byUser;
	node["method"] = transform.method;
	node["program"] = transform.program;
	node["details"] = transform.details;

	node["time"] = transform.time;

	node["history"] = Json::Value(Json::arrayValue);
	for(std::list<sttl::Transform::Id>::const_iterator itr = transform.history.begin(); itr != transform.history.end(); ++itr){
		node["history"].append(itr->toString());
	}

	return true;
}


uint64_t hash_combine(uint64_t& seed, const uint64_t &hash) {
    seed ^= hash + 0x9e3773e79b4319b9 + (seed<<6) + (seed>>2);
    return seed;
}

// inline void hash_combine(std::size_t& seed, const std::size_t &hash) {
//     seed ^= hash + 0x9e3779b9 + (seed<<6) + (seed>>2);
// }

bool sttl::Transform::generateId(){
	if(!(id==Id())){
		std::cerr<<"Id was not empty when calling generateId()!"<<std::endl;
		return false;
	}

	uint64_t id_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	uint64_t hashNames = 0;
	hash_combine(hashNames, std::hash<std::string>()(frameName));
	hash_combine(hashNames, std::hash<std::string>()(referenceFrameName));
	hash_combine(hashNames, std::hash<std::string>()(byUser));
	hash_combine(hashNames, std::hash<std::string>()(method));
	hash_combine(hashNames, std::hash<std::string>()(program));

	uint64_t hashTransf = 0;
	for(int i=0; i<4; ++i) for(int k=0; k<4; ++k){
		hash_combine(hashTransf, std::hash<double>()(transform(i, k)));
	}

	static bool srandCalled = false;
	if(!srandCalled){
		srandCalled = true;
		std::srand(id_time);
	}
	uint64_t random = std::rand();
	hash_combine(random, std::rand());
	hash_combine(random, std::rand());
	hash_combine(random, std::rand());
	hash_combine(random, std::rand());

	id = Id(hash_combine(random, id_time), hash_combine(hashNames, hashTransf));
	return true;
}


bool sttl::saveList(const std::list<sttl::Transform> & transfs, const std::string &file ){
	Json::Value root(Json::arrayValue);
	for(std::list<sttl::Transform>::const_iterator itr = transfs.begin(); itr != transfs.end(); ++itr){
		Json::Value frame;
		fillJson(*itr, frame);
		root.append(frame);
	}
	std::fstream fs;
	fs.open (file, std::fstream::out);
    if (fs.is_open()){

		fs<<root;

		fs.close();
	}else{
		cerr<<"Unable to open file "<<file<<" for writing..."<<std::endl;
		return false;
	}

	return true;
}


bool sttl::loadList(std::list<sttl::Transform> & transfs, const std::string &file ){
	if(!transfs.empty()){
		std::cerr<<"Load list - list provided not empty!"<<std::endl;
		return false;
	}

	std::fstream fs;
	fs.open (file, std::fstream::in);
    if (!fs.is_open()){
    	std::cerr<<"Error opening file "<<file<<std::endl;
    	return false;
    }
    std::cerr<<" reading JSON from "<<file<<std::endl;
	Json::Value root;
	fs >> root;

	std::cerr<<" iterating through JSON tree ..."<<std::endl;
    for(int i = 0 ; i < root.size() ; ++i){
    	sttl::Transform trans;
		std::cerr<<" reading "<<i<<std::endl;
    	bool ret = readJson(trans, root[i]);
		std::cerr<<" done reading "<<i<<std::endl;
    	if(!ret){
    		std::cerr<<"Error reading transform # "<<i<<std::endl;
    		return false;
    	}
    	transfs.push_back(trans);
    }
    return true;
}

std::list<sttl::Frame>::iterator sttl::findFrameForData(std::list<sttl::Frame> &frames, const std::string & dataName){
	for(std::list<sttl::Frame>::iterator itr = frames.begin(); itr!= frames.end(); ++itr){
		std::string cmpName = dataName;
		// adjust the size of the data name such that it is max as long as the frame name here
		if(cmpName.size()>itr->name.size()){
			cmpName = cmpName.substr(0, itr->name.size());
		}
		if(itr->name.compare(cmpName) == 0 ){
			return itr;
		}
	}
	return frames.end();
}

std::list<sttl::Frame>::iterator sttl::findFrame(std::list<sttl::Frame> &frames, const std::string & frame){
	for(std::list<sttl::Frame>::iterator itr = frames.begin(); itr!= frames.end(); ++itr){
		if(itr->name.compare(frame) == 0 ){
			return itr;
		}
	}
	return frames.end();
}

/**
 * If not already in the frames list add new frames
 *
 */
void addFrames(std::list<sttl::Transform> & transforms, std::list<sttl::Frame> &frames){
	for(std::list<sttl::Transform>::iterator itr = transforms.begin(); itr != transforms.end(); ++itr){
		if(findFrame(frames, itr->frameName) == frames.end() ){
			sttl::Frame frame;
			frame.name = itr->frameName;
			frames.push_back(frame);
		}
		if(findFrame(frames, itr->referenceFrameName) == frames.end() ){
			sttl::Frame frame;
			frame.name = itr->referenceFrameName;
			frames.push_back(frame);
		}
	}
}

std::list<sttl::Transform>::iterator findTransform(std::list<std::list<sttl::Transform>::iterator> & transfs, const sttl::Transform::Id &id){
	for(std::list<std::list<sttl::Transform>::iterator>::iterator itr = transfs.begin(); itr!=transfs.end(); ++itr){
		if((*itr)->id == id) return *itr;
	}
	return std::list<sttl::Transform>::iterator();
}

/**
 * Add those transforms to their respective frames (but not two with the same id)
 * Assumes that the frame exists!
 *
 */
void addTransforms(std::list<sttl::Transform> & transforms, std::list<sttl::Frame> &frames){
	for(std::list<sttl::Transform>::iterator itr = transforms.begin(); itr != transforms.end(); ++itr){
		std::list<sttl::Frame>::iterator frame = findFrame(frames, itr->frameName);
		std::list<sttl::Frame>::iterator reference = findFrame(frames, itr->referenceFrameName);

		if(findTransform(frame->parents, itr->id) == std::list<sttl::Transform>::iterator()){
			// no transform found - add it!
			frame->parents.push_back(itr);
		}
		if(findTransform(reference->children, itr->id) == std::list<sttl::Transform>::iterator()){
			// no transform found - add it!
			reference->children.push_back(itr);
		}
	}
}

/** sets the visited boolean in all frames to false
 */
void resetVisited(std::list<sttl::Frame> &frames){
	for(std::list<sttl::Frame>::iterator itr = frames.begin(); itr!= frames.end(); ++itr){
		itr->visited = false;
	}
}

void calculateCoordinates(std::list<sttl::Frame> &frames, std::list<std::list<sttl::Transform>::iterator> &wavefront){

	while(!wavefront.empty()){
		std::list<sttl::Transform>::iterator trans = wavefront.front();
		wavefront.pop_front();
		std::list<sttl::Frame>::iterator frame = findFrame(frames, trans->frameName);
		std::list<sttl::Frame>::iterator reference = findFrame(frames, trans->referenceFrameName);
		if(frame->visited) continue;
		if(!reference->visited){
			std::cerr<<"Reference not visited - should not happen!!!"<<std::endl;
			continue;
		}
		frame->rootTransform = reference->rootTransform * trans->transform;
		frame->visited = true;
		wavefront.insert(wavefront.end(), frame->children.begin(), frame->children.end());
	}

}

bool sttl::generateFrames(std::list<sttl::Transform> & transforms, std::list<sttl::Frame> &frames){

	cerr<<"frames size before: "<<frames.size()<<endl;
	addFrames(transforms, frames);
	cerr<<"frames size after: "<<frames.size()<<endl;
	cerr<<"transforms size : "<<transforms.size()<<endl;

	addTransforms(transforms, frames);

	resetVisited(frames);

	std::list<sttl::Frame>::iterator root = findFrame(frames, "root");

	if(root == frames.end()){
		cerr<<"Root frame not found!"<<endl;
		return false;
	}

	cerr<<" Root parents (should be 0): "<<root->parents.size()<<"; root children: "<<root->children.size()<<std::endl;

	std::list<std::list<Transform>::iterator> wavefront = root->children;

	root->rootTransform = Eigen::Transform<double,3,Eigen::Affine>::Identity();
	root->visited = true;

	calculateCoordinates(frames, wavefront);

	return true;
}



