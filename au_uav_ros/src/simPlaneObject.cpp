#include "au_uav_ros/simPlaneObject.h"
using namespace au_uav_ros;

#define MAXIMUM_TURNING_ANGLE 22.5 //degrees

SimPlaneObject::SimPlaneObject(void) : PlaneObject() {
	simSpeed = 1;
}

SimPlaneObject::SimPlaneObject(int _id, struct waypoint wp) : PlaneObject(_id, wp) {
	/* TODO Why does wp index need to be -1 on start vs 0 */
	this->speed = MPH_SPEED;
	simSpeed = 1;
}

void SimPlaneObject::setSimSpeed(double _simSpeed) {
	simSpeed = _simSpeed; //TODO guard input if negatives don't work
}

bool SimPlaneObject::simulate(double duration, au_uav_ros::Telemetry *telem) {
	fillTelemetryUpdate(duration, telem);
	return true;
}

bool SimPlaneObject::handleNewCommand(au_uav_ros::Command newCommand) {
	
	if(this->id != newCommand.planeID)
	{
		return false;
	}
	if (newCommand.commandID == COMMAND_AVOID_WP)  {
		//this->destination.latitude = newCommand.latitude;
		//this->destination.longitude = newCommand.longitude;
		//this->destination.altitude = newCommand.altitude;
		waypoint wp;
		wp.latitude = newCommand.latitude;
		wp.longitude = newCommand.longitude;
		wp.altitude = newCommand.altitude;
		if (avoidancePath.size() > 0) {
			avoidancePath.pop_front();
		}
		avoidancePath.push_front(wp);
	} else if (newCommand.commandID == COMMAND_NORMAL_WP) {
		waypoint wp;
		wp.latitude = newCommand.latitude;
		wp.longitude = newCommand.longitude;
		wp.altitude = newCommand.altitude;
		normalPath.push_back(wp);
	}
	
	this->currentWaypointIndex++;
	
	return true;
}

bool SimPlaneObject::fillTelemetryUpdate(double duration, au_uav_ros::Telemetry *update) {
	//difference in latitudes in radians
	double lat1 = currentLoc.latitude*DEGREES_TO_RADIANS;
	double lat2, long2;
	if (avoidancePath.size() > 0) {
		lat2 = avoidancePath.front().latitude*DEGREES_TO_RADIANS;
		long2 = avoidancePath.front().longitude*DEGREES_TO_RADIANS;
	} else {
		lat2 = normalPath.front().latitude*DEGREES_TO_RADIANS;
		long2 = normalPath.front().longitude*DEGREES_TO_RADIANS;
	}
	double long1 = currentLoc.longitude*DEGREES_TO_RADIANS;
	
	double deltaLat = lat2 - lat1;
	double deltaLong = long2 - long1;
	
	//haversine crazy math, should probably be verified further beyond basic testing
	//calculate distance from current position to destination
	double a = pow(sin(deltaLat / 2.0), 2);
	a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
	double c = 2.0 * asin(sqrt(a));
	this->distanceToDestination = EARTH_RADIUS * c;
	
	//calculate bearing from current position to destination
	double y = sin(deltaLong)*cos(lat2);
	double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltaLong);
	this->targetBearing = atan2(y, x)*RADIANS_TO_DEGREES;
	
	//make sure we're actually traveling somewhere
	if(this->currentWaypointIndex >= 0)
	{
		//calculate the real bearing based on our maximum angle change
		//first create a temporary bearing that is the same as bearing but at a different numerical value
		double tempBearing = -1000;
		if((this->targetBearing) < 0)
		{
			tempBearing = this->targetBearing + 360;
		}
		else
		{
			tempBearing = this->targetBearing - 360;
		}
		
		double diff1 = abs(this->currentBearing - this->targetBearing);
		double diff2 = abs(this->currentBearing - tempBearing);
	
		//check for easy to calculate values first
		if(diff1 < (MAXIMUM_TURNING_ANGLE*simSpeed*duration) || diff2 < (MAXIMUM_TURNING_ANGLE*simSpeed*duration))
		{
			//the difference is less than our maximum angle, set it to the bearing
			this->currentBearing = this->targetBearing;
		}
		else
		{
			//we have a larger difference than we can turn, so turn our maximum
			double mod;
			if(diff1 < diff2)
			{
				if(this->targetBearing > this->currentBearing) mod = (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
				else mod = 0 - (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
			}
			else
			{
				if(tempBearing > this->currentBearing) mod = (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
				else mod = 0 - (MAXIMUM_TURNING_ANGLE*simSpeed*duration);
			}
		
			//add our mod, either +22.5 or -22.5
			this->currentBearing = this->currentBearing + mod;
		
			//tweak the value to keep it between -180 and 180
			if(this->currentBearing > 180) this->currentBearing = this->currentBearing - 360;
			if(this->currentBearing <= -180) this->currentBearing = this->currentBearing + 360;
		}
	
		//time to calculate the new positions, God help us
		/*
		Algorithm for updating position:
		1) Estimate new latitude using basic trig and this equation:
		   lat2 = lat1 + (MPS_SPEED*cos(bearing))*METERS_TO_LATITUDE
		2) Use law of haversines to find the new longitude
		   haversin(c) = haversin(a-b) + sin(a)*sin(b)*haversin(C)
		   where haversin(x) = (sin(x/2.0))^2
		   where c = MPS_SPEED/EARTH_RADIUS (radians)
		   where a = 90 - lat1 (degrees)
		   where b = 90 - lat2 (degrees)
		   where C = the change in longitude, what we are solving for
		   
		   C = 2.0 * arcsin(sqrt((haversin(c) - haversin(a-b))/(sin(a)*sin(b))))
		*/
	
		//1) Estimate new latitude using basic trig and this equation
		this->currentLoc.latitude = lat1*RADIANS_TO_DEGREES + ((MPS_SPEED*duration*simSpeed)*cos(this->currentBearing*DEGREES_TO_RADIANS))*METERS_TO_LATITUDE;
		//this->currentLoc.latitude = lat1*RADIANS_TO_DEGREES + (MPS_SPEED*cos(this->currentBearing*DEGREES_TO_RADIANS))*METERS_TO_LATITUDE;
		
		//2) Use the law of haversines to find the new longitude
		double temp = pow(sin(((MPS_SPEED*simSpeed*duration)/EARTH_RADIUS)/2.0), 2); //TODO verify
		//double temp = 7.69303281*pow(10, -13); //always the same, see above calculation
		temp = temp - pow(sin((this->currentLoc.latitude*DEGREES_TO_RADIANS - lat1)/2.0), 2);
		temp = temp / (sin(M_PI/2.0 - lat1)*sin((M_PI/2.0)-this->currentLoc.latitude*DEGREES_TO_RADIANS));
		temp = 2.0 * RADIANS_TO_DEGREES * asin(sqrt(temp));
		
		//depending on bearing, we should be either gaining or losing longitude
		if(currentBearing > 0)
		{
			this->currentLoc.longitude += temp;
		}
		else
		{
			this->currentLoc.longitude -= temp;
		}
	}
		
	//fill out the actual data
	update->planeID = this->id;
	update->currentLatitude = this->currentLoc.latitude;
	update->currentLongitude = this->currentLoc.longitude;
	update->currentAltitude = this->currentLoc.altitude;

	//COLLISION_THRESHOLD is 12 meters - defined in standardDefs.h

	//first see if we need to dump any points from the avoidance path (dump wp if within 2s of it)
	if(!avoidancePath.empty() && distanceToDestination > -COLLISION_THRESHOLD && distanceToDestination < COLLISION_THRESHOLD)
	{
		// now check if planeDest has been updated
		//if (distanceBetween(avoidancePath.front(), getDestination()) < COLLISION_THRESHOLD)
			//this means we met the normal path's first point, so pop it
			avoidancePath.pop_front();

		//if we pop'd a wp, and there is another wp in avoidancePath, command will be true.
		//2/26/2013-might need to always be true, but not sure now.
		//if (!avoidancePath.empty()) isCommand = true;
	}
	//if here then destination is in normalPath
	//next see if we need to dump any points from the normal path (dump wp if within 1s of it)
	else if(!normalPath.empty() && distanceToDestination > -COLLISION_THRESHOLD && distanceToDestination < COLLISION_THRESHOLD)
	{
		// now check if planeDest has been updated
		//if (distanceBetween(normalPath.front(), getDestination()) < COLLISION_THRESHOLD)
			//this means we met the normal path's waypoint, so pop it
			if (normalPath.size() > 1) {
				normalPath.pop_front();
			}
		//if we pop'd a wp, and there is another wp in normalPath, command is true.
		//if (!normalPath.empty()) isCommand = true;
	}

	//determine which point we should be going to right now
	//first, check the avoidance queue, since survival is priority #1
	//Logic here:  If we have any waypoints in avoidance, direct there.  
	//	       Otherwise, check for a new normal waypoint and send it if new.
	/**
	if(!avoidancePath.empty())
	{
		destination = avoidancePath.front();
		//isCommand = true;
		//isAvoid = true;
	}
	
	//avoidance queue is empty so check normal pathing
	//if we have hit a normal wp (isCommand == true) or a new simulated UAV is active
	else if(!normalPath.empty() && (isCommand || (msg.currentWaypointIndex == -1 && planeDest.latitude == 0 && planeDest.longitude == 0)))
	{
		destination = normalPath.front();
		isCommand = true;
		isAvoid = false;
	} */
	
	if (avoidancePath.size() > 0) {
		waypoint wp = avoidancePath.front();
		update->destLatitude = wp.latitude;
		update->destLongitude = wp.longitude;
		update->destAltitude = wp.altitude;
	} else {
		waypoint wp = normalPath.front();
		update->destLatitude = wp.latitude;
		update->destLongitude = wp.longitude;
		update->destAltitude = wp.altitude;
	}
	
	update->groundSpeed = this->speed; //TODO if ripna assumes ground speed is per 1 second then mulitply by duration
	update->targetBearing = this->targetBearing;
	
	update->currentWaypointIndex = this->currentWaypointIndex;
	update->distanceToDestination = this->distanceToDestination;
	update->telemetryHeader.seq = this->updateIndex++;
	update->telemetryHeader.stamp = ros::Time::now();
	/*TODO make sure sims isn't assuming 1 sec freq */ updateTime();
	
	return true;
}
