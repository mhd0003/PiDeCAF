#include "au_uav_ros/plane.h"
using au_uav_ros::Telemetry;
using au_uav_ros::Vector2D;
using au_uav_ros::waypoint;
using ipn::Plane;

Plane::Plane() : planeID(0), currentBearing(0), groundSpeed(0) {
	//
}

Plane::Plane(const Telemetry &update) : planeID(update.planeID),
	currentBearing(0), groundSpeed(update.groundSpeed), lastUpdate(update) {
	//
	currentLocation.latitude = update.currentLatitude;
	currentLocation.longitude = update.currentLongitude;
	currentLocation.altitude = update.currentAltitude;
	currentLocation.planeID = update.planeID;
}


int Plane::getID() const {
	return planeID;
}

double Plane::getCurrentBearing() const {
	return currentBearing;
}

double Plane::getGroundSpeed() const {
	return groundSpeed;
}

waypoint Plane::getCurrentLocation() const {
	return currentLocation;
}

waypoint Plane::getAvoidanceWaypoint() const {
	return avoidWP;
}

waypoint Plane::getGoalWaypoint() const {
	return goalWP;
}

waypoint Plane::getDestinationWaypoint() const {
	if (avoidWP != INVALID_GPS_WP) {
		return avoidWP;
	} else if (goalWP != INVALID_GPS_WP) {
		return goalWP;
	} else {
		return INVALID_GPS_WP;
	}
}

std::map<int, Plane> Plane::getPlaneMap() {
	return planeMap;
}


void Plane::setID(int idIn) {
	planeID = idIn;
}

void Plane::setAvoidanceWaypoint(const waypoint &wpIn) {
	avoidWP = wpIn;
}
void Plane::setGoalWaypoint(const waypoint &wpIn) {
	goalWP = wpIn;
}

void Plane::update(const Telemetry &update) {
	if (this->planeID == update.planeID) {
		updateThisPlane(update);
	} else {
		updatePlaneMap(update);
	}
}

void Plane::updateThisPlane(const Telemetry &update) {
	currentBearing = update.targetBearing;
	updatePosition(update);

	groundSpeed = update.groundSpeed;
	// lastUpdate = update;

	// waypoint destination;
	// destination.latitude = update.destLatitude;
	// destination.longitude = update.destLongitude;
	// destination.altitude = update.destAltitude;

	// if (avoidWP != INVALID_GPS_WP && update.distanceToDestination < COLLISION_THRESHOLD) {
	// 	avoidWP = INVALID_GPS_WP;
	// }
	// else if (goalWP != INVALID_GPS_WP && update.distanceToDestination < COLLISION_THRESHOLD) {
	// 	goalWP = INVALID_GPS_WP;
	// }

	groundSpeed = update.groundSpeed;
	lastUpdate = update;
}

void Plane::updatePlaneMap(const Telemetry &update) {
	if (planeMap.find(update.planeID) != planeMap.end()) {
		planeMap[update.planeID].update(update);
	} else {
		Plane newPlane(update);
		planeMap[update.planeID] = newPlane;
	}
}

void Plane::updatePosition(const Telemetry &update) {
	// waypoint previousLocation = currentLocation;
	currentLocation.latitude = update.currentLatitude;
	currentLocation.longitude = update.currentLongitude;
	currentLocation.altitude = update.currentAltitude;

	// if (previousLocation != currentLocation) {
	// 	Vector2D positionUpdate(previousLocation, currentLocation);
	// 	currentBearing = positionUpdate.getAngle();
	// }

}
