/* Implementation of planeObject.h

*/

#include "ros/ros.h"
#include "au_uav_ros/planeObject.h"
#include <math.h>
#include "au_uav_ros/standardFuncs.h" /* for PI, EARTH_RADIUS in meters */
//#include "AU_UAV_ROS/ForceField.h"
#include <math.h>
using namespace au_uav_ros;


/* Implementation of the default constructor: Member variables are set to zero */
PlaneObject::PlaneObject() {
	this->id = 0;
	this->currentLoc.altitude = 0.0;
	this->currentLoc.latitude = 0.0;
	this->currentLoc.longitude = 0.0;
	this->previousLoc.altitude = 0.0;
	this->previousLoc.latitude = 0.0;
	this->previousLoc.longitude = 0.0;
	this->targetBearing = 0.0;
	this->currentBearing = 0.0;

	this->speed = 0.0;
	//this->destination.latitude = 0.0;
	//this->destination.longitude = 0.0;
	//this->destination.altitude = 0.0;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->tempForceWaypoint = destination;
	this->collisionRadius = 0.0;
	this->setField(0,0); //initialize field to default configuration
	//this->planesToAvoid = new std::map<int, PlaneObject>();
	planesToAvoid.clear();
}

PlaneObject::PlaneObject(int _id, struct waypoint wp) {
	this->id = _id;
	this->currentLoc.altitude = 0.0;
	this->currentLoc.latitude = 0.0;
	this->currentLoc.longitude = 0.0;
	this->previousLoc.altitude = 0.0;
	this->previousLoc.latitude = 0.0;
	this->previousLoc.longitude = 0.0;
	this->targetBearing = 0.0;
	this->currentBearing = 0.0;

	this->speed = 0.0;
	this->collisionRadius = 0.0;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->normalPath.push_back(wp);
}

PlaneObject::PlaneObject(int _id) {
	this->id = _id;
}

/* Explicit value constructor using Telemetry */
PlaneObject::PlaneObject(double cRadius, const Telemetry &msg) { 
	this->id = msg.planeID;
	this->currentLoc.altitude = msg.currentAltitude;
	this->currentLoc.latitude = msg.currentLatitude;
	this->currentLoc.longitude = msg.currentLongitude;
	this->previousLoc.altitude = 0.0;
	this->previousLoc.latitude = 0.0;
	this->previousLoc.longitude = 0.0;
	this->targetBearing = msg.targetBearing;
	this->currentBearing = 0.0;

	this->speed = msg.groundSpeed;
	//this->destination.latitude = msg.destLatitude;
	//this->destination.longitude = msg.destLongitude;
	//this->destination.altitude = msg.destAltitude;
	waypoint wp;
	wp.latitude = msg.destLatitude;
	wp.longitude = msg.destLongitude;
	wp.altitude = msg.destAltitude;
	this->normalPath.push_back(wp);
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = cRadius;
	this->setField(0,0); //initialize field to default configuration
	//this->planesToAvoid = new std::map<int, PlaneObject>();
	planesToAvoid.clear();
}

/* mutator functions to update member variables */
void PlaneObject::setID(int id){
	this->id = id;
}

void PlaneObject::setPreviousLoc(double lat, double lon, double alt) {
	this->previousLoc.latitude = lat;
	this->previousLoc.longitude = lon;
	this->previousLoc.altitude = alt;
}

void PlaneObject::setCurrentLoc(double lat, double lon, double alt) {
	this->currentLoc.latitude = lat;
	this->currentLoc.longitude = lon;
	this->currentLoc.altitude = alt;
}

void PlaneObject::setTargetBearing(double tBearing) {
	this->targetBearing = tBearing;
}

void PlaneObject::setCurrentBearing(double cBearing) {
	this->currentBearing = cBearing;
}

void PlaneObject::setSpeed(double speed) {
	this->speed = speed;
}
//should only be called for avoidance wps
void PlaneObject::setDestination(const waypoint &destination) {
	this->avoidancePath.clear(); //TODO find out if we need queue or just 1 wp
	this->avoidancePath.push_front(destination);
}

void PlaneObject::setTempForceWaypoint(const waypoint &tempForceWaypoint){
	this->tempForceWaypoint = tempForceWaypoint;
}

void PlaneObject::updateTime(void) {
	this->lastUpdateTime = ros::Time::now().toSec();
}


bool PlaneObject::update(const Telemetry &msg, Command &newCommand) {
	//Update previous and current position
	this->setPreviousLoc(this->currentLoc.latitude, this->currentLoc.longitude, this->currentLoc.altitude);
	this->setCurrentLoc(msg.currentLatitude, msg.currentLongitude, msg.currentAltitude);
	
	//Calculate actual Cardinal Bearing
	double numerator = (this->currentLoc.latitude - this->previousLoc.latitude);
	double denominator = (this->currentLoc.longitude - this->previousLoc.longitude);
	double angle = atan2(numerator*DELTA_LAT_TO_METERS,denominator*DELTA_LON_TO_METERS)*180/PI;

	if (this->currentLoc.latitude != this->previousLoc.latitude && this->currentLoc.longitude != this->previousLoc.longitude) //TODO change to ||
	{ 
			this->setCurrentBearing(toCardinal(angle));
	}
	else this->setCurrentBearing(0.0);

	// Update everything else
	this->setTargetBearing(msg.targetBearing);
	this->setSpeed(msg.groundSpeed);
	this->updateTime();

	//this bool is set true only if there is something available to be sent to the UAV
	bool isCommand = false;

	//this bool is set if the command available is an avoidance maneuver
	bool isAvoid;

	//store the last update
	//this->latestUpdate = update;

	//data structs to be used by the coordinator
	struct waypoint destination;
	struct waypoint current;
	struct waypoint planeDest;

	current.latitude = msg.currentLatitude;
	current.longitude = msg.currentLongitude;
	current.altitude = msg.currentAltitude;

	planeDest.latitude = msg.destLatitude;
	planeDest.longitude = msg.destLongitude;
	planeDest.altitude = msg.destAltitude;

	//COLLISION_THRESHOLD is 12 meters - defined in standardDefs.h

	//first see if we need to dump any points from the avoidance path (dump wp if within 2s of it)
	if(!avoidancePath.empty() && msg.distanceToDestination > -COLLISION_THRESHOLD && msg.distanceToDestination < COLLISION_THRESHOLD)
	{
		// now check if planeDest has been updated
		//if (distanceBetween(avoidancePath.front(), planeDest) < COLLISION_THRESHOLD)
			//this means we met the normal path's first point, so pop it
			avoidancePath.pop_front();

		//if we pop'd a wp, and there is another wp in avoidancePath, command will be true.
		//2/26/2013-might need to always be true, but not sure now.
		if (!avoidancePath.empty()) isCommand = true;
	}
	//if here then destination is in normalPath
	//next see if we need to dump any points from the normal path (dump wp if within 1s of it)
	else if(!normalPath.empty() && msg.distanceToDestination > -COLLISION_THRESHOLD && msg.distanceToDestination < COLLISION_THRESHOLD)
	{
		// now check if planeDest has been updated
		//if (distanceBetween(normalPath.front(), planeDest) < COLLISION_THRESHOLD)
			//this means we met the normal path's waypoint, so pop it
			normalPath.pop_front();
		//if we pop'd a wp, and there is another wp in normalPath, command is true.
		if (!normalPath.empty()) isCommand = true;
	}

	//determine which point we should be going to right now
	//first, check the avoidance queue, since survival is priority #1
	//Logic here:  If we have any waypoints in avoidance, direct there.  
	//	       Otherwise, check for a new normal waypoint and send it if new.
	if(!avoidancePath.empty())
	{
		destination = avoidancePath.front();
		newCommand.commandID = COMMAND_AVOID_WP;
		isCommand = true;
		isAvoid = true;
	}
	
	//avoidance queue is empty so check normal pathing
	//if we have hit a normal wp (isCommand == true) or a new simulated UAV is active
	else if(!normalPath.empty() && (isCommand || (msg.currentWaypointIndex == -1 && planeDest.latitude == 0 && planeDest.longitude == 0)))
	{
		destination = normalPath.front();
		newCommand.commandID = COMMAND_NORMAL_WP;
		isCommand = true;
		isAvoid = false;
	}
	
	//if we have a command to process, process it
	if(isCommand)
	{
		//the current waypoint is incorrect somehow, send corrective command
		//newCommand.commandHeader.seq = this->commandIndex++;
		newCommand.commandHeader.stamp = ros::Time::now();
		newCommand.planeID = id;//this->latestUpdate.planeID;
		newCommand.latitude = destination.latitude;
		newCommand.longitude = destination.longitude;
		newCommand.altitude = destination.altitude;
		return true;
	}
		
	else {
		//if we get here, then avoidance queue is empty and no new wps need to be sent for normal path
		//ROS_INFO("Plane #%d has no commands right now.", this->latestUpdate.planeID);
		return false;
	}
}


void PlaneObject::setField(ForceField  newField){
	planeField = newField;
}


/* accessor functions */
int PlaneObject::getID(void) const {
	return this->id;
}

waypoint PlaneObject::getPreviousLoc(void) const {
	return this->previousLoc;
}

waypoint PlaneObject::getCurrentLoc(void) const {
	return this->currentLoc;
}

double PlaneObject::getTargetBearing(void) const {
	return this->targetBearing;
}

double PlaneObject::getCurrentBearing(void) const {
	return this->currentBearing;
}
	
double PlaneObject::getSpeed(void) const {
	return this->speed;
}

double PlaneObject::getLastUpdateTime(void) const {
	return this->lastUpdateTime;
}

/*
int PlaneObject::getCommandIndex(void) const {
	return this->commandIndex;
}
*/

waypoint PlaneObject::getDestination(void) const {
	if (avoidancePath.size() > 0 ) {
		return avoidancePath.front();
	} else {
		return normalPath.front();
	}
}

/* Find distance between this plane and another plane, returns in meters */
double PlaneObject::findDistance(const PlaneObject& plane) const {
	return this->findDistance(plane.currentLoc.latitude, plane.currentLoc.longitude);
}


/* Find distance between this plane and another pair of coordinates, 
returns value in meters */
double PlaneObject::findDistance(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;

	return sqrt(pow(xdiff, 2) + pow(ydiff, 2));
}

/* Find Cartesian angle between this plane and another plane, using this plane
as the origin */
double PlaneObject::findAngle(const PlaneObject& plane) const {
	return this->findAngle(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find Cartesian angle between this plane and another plane's latitude/longitude 
using this plane as the origin */
double PlaneObject::findAngle(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;

	//Get angle in degrees, in range [-180 to 180] in cartesian coordinate frame
	double degrees =  atan2(ydiff, xdiff)*180.0/PI;

	//added in an attempt to get test to work
	return degrees;

}

//FIELD METHODS


/*This method will adjust the field of the plane to specifications provided by the arguments
 * TODO:
 * 		DELETE PREVIOUS FIELD
 * 		Enable choosing multiple field setups, this method will currently only call one field type
 */
void PlaneObject::setField(int encodedFieldShape, int encodedFieldFunction){

}

ForceField PlaneObject::getField(){
	return this->planeField;
}
/*
double PlaneObject::findMyFieldForceMagnitude(fsquared::relativeCoordinates relativePosition){
	return planeField->findFieldForceMagnitude(relativePosition);
}


bool PlaneObject::isInMyField(fsquared::relativeCoordinates relativePosition, double fieldAngle){
	return planeField->isrelativeCoordinatesInMyField(relativePosition, fieldAngle);
}
*/


/*Accessor method for planesToAvoid map */

std::map<int, PlaneObject> & PlaneObject::getMap()	{
	return planesToAvoid; 		
}

/* 
* 
*Insert plane. Copy by value. If it exists, will update the existing plane.
*After adding plane, will clear its map to prevent infinite loop of planes with maps with planes...
*
*
*/
void PlaneObject::planeIn_updateMap(PlaneObject plane)	{
	(planesToAvoid)[plane.getID()]  = plane;
	plane.clearMap();
}

/*
 * Empties plane's map of other planes which is exerting a force.
 */
void PlaneObject::clearMap()	{
	planesToAvoid.clear();
}

/* Ensure plane is not in the map */
void PlaneObject::planeOut_updateMap(PlaneObject &plane)	{

	//If plane is in map, TAKE IT OUT arggghh
	std::map<int, PlaneObject> ::iterator it;
	int plane_id = plane.getID();
	it = planesToAvoid.find(plane_id);
	if(it != planesToAvoid.end())
		planesToAvoid.erase(it);
}

// TODO: Add equality check for force field
PlaneObject& PlaneObject::operator=(const PlaneObject& plane) {
	if(this == &plane)
	        return *this;
	this->id = plane.id;
	this->currentLoc.altitude = plane.currentLoc.altitude;
	this->currentLoc.latitude = plane.currentLoc.latitude;
	this->currentLoc.longitude = plane.currentLoc.longitude;

	this->previousLoc.altitude = plane.previousLoc.altitude;
	this->previousLoc.latitude = plane.previousLoc.latitude;
	this->previousLoc.longitude = plane.previousLoc.longitude;

	//this->destination.latitude = plane.destination.latitude;
	//this->destination.longitude = plane.destination.longitude;
	//this->destination.altitude = plane.destination.latitude;

	this->targetBearing = plane.targetBearing;
	this->currentBearing = plane.currentBearing;

	this->speed = plane.speed;
	this->lastUpdateTime = plane.lastUpdateTime;
	this->collisionRadius = plane.collisionRadius;

	return *this;
}

void PlaneObject::addWp(struct waypoint wp, bool normal) {
	if (normal) {
		normalPath.push_back(wp);
	} else {
		avoidancePath.push_back(wp);
	}
}

void PlaneObject::removeWp(struct waypoint wp, bool normal) {
	if (normal) {
		//normalPath.remove(wp);
		std::list<waypoint>::iterator i;
		for (i = normalPath.begin(); i != normalPath.end(); i++) {
			if (wp == *i) {
				break;
			}
		}
		if (i != normalPath.end()) {
			normalPath.erase(i);	
		}
	} else {
		//avoidancePath.remove(wp);
		std::list<waypoint>::iterator i;
		for (i = avoidancePath.begin(); i != avoidancePath.end(); i++) {
			if (wp == *i) {
				break;
			}
		}
		if (i != avoidancePath.end()) {
			avoidancePath.erase(i);	
		}
	}
}

Command PlaneObject::getPriorityCommand(void) {
	//start with defaults
	Command ret;
	ret.planeID = -1;
	ret.latitude = -1000;
	ret.longitude = -1000;
	ret.altitude = -1000;

	//check avoidance queue
	if(!avoidancePath.empty())
	{
		//we have an avoidance point, get that one
		ret.latitude = avoidancePath.front().latitude;
		ret.longitude = avoidancePath.front().longitude;
		ret.altitude = avoidancePath.front().altitude;
		ret.commandID = COMMAND_AVOID_WP; 
	}
	else
	{
		if(!normalPath.empty())
		{
			//we have a normal path point at least, fill it out
			ret.latitude = normalPath.front().latitude;
			ret.longitude = normalPath.front().longitude;
			ret.altitude = normalPath.front().altitude;
			ret.commandID = COMMAND_NORMAL_WP;
		}
		else
		{
			//normal path is also empty do nothing
		}
	}

	//fill out our header and return this bad boy
	//ret.commandHeader.seq = this->commandIndex++;
	ret.commandHeader.stamp = ros::Time::now();
	return ret;
}
