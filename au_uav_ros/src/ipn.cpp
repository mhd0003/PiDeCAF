#include "au_uav_ros/ipn.h"
using namespace au_uav_ros;

// Print greatest threat
#define IPN_PRINT_DEBUG_0 true
// Print each threat's info
#define IPN_PRINT_DEBUG_1 true
// Print each threat's info as ripna.cpp would calculate it
#define IPN_PRINT_RIPNA_1 false
// Print turning info
#define IPN_PRINT_DEBUG_2 false
// Print turning info as ripna.cpp would calculate it
#define IPN_PRINT_RIPNA_2 false
// Print waypoint info
#define IPN_PRINT_DEBUG_3 false
// Print waypoint info as ripna.cpp would calculate it
#define IPN_PRINT_RIPNA_3 false

#define MAXIMUM_TURNING_ANGLE 22.5 //degrees


Vector2D ipn::getSeparationVector(const PlaneObject &plane1, const PlaneObject &plane2) {
	return Vector2D(plane1.getCurrentLoc(), plane2.getCurrentLoc());
}

Vector2D ipn::getDirectionVector(const PlaneObject &plane1, const PlaneObject &plane2) {
	Vector2D d_1(cos(plane1.getCurrentBearing()*DEGREES_TO_RADIANS),
		sin(plane1.getCurrentBearing()*DEGREES_TO_RADIANS));
	Vector2D d_2(cos(plane2.getCurrentBearing()*DEGREES_TO_RADIANS),
		sin(plane2.getCurrentBearing()*DEGREES_TO_RADIANS));

	return d_1 - d_2;
}

/*
* (1) Get threat info for all planes
* (2) Determine greatest threat (if any)
* (3) Generate avoidance waypoint
*/
bool ipn::checkForThreats(PlaneObject &thisPlane, std::map<int, PlaneObject> &allPlanes, waypoint &avoidanceWP) {
	/* Set threshold values for this plane's speed */
	SEPARATION_THRESHOLD = thisPlane.getSpeed() * 10.0;
	ZEM_THRESHOLD = thisPlane.getSpeed() * 3.5;

	std::vector<threatInfo> allThreats;

	/* (1) Get threat info for all planes */
	std::map<int, PlaneObject>::iterator it;
	for (it = allPlanes.begin(); it != allPlanes.end(); it++) {
		if (thisPlane.getID() == it->first) {
			ROS_WARN("$!$!$!$ Plane in my map with same ID! $!$!$!$");
			continue;
		}

		allThreats.push_back(getThreatInfo(thisPlane, it->second));
	}

	/* (2) Determine greatest threat */
	threatInfo greatestThreat = findGreatestThreat(allThreats);

	if (greatestThreat.threatPlane == NULL) {
		return false;
	}

	/* (3) Generate avoidance waypoint */
	avoidanceWP = createAvoidanceWaypoint(thisPlane, greatestThreat);
	avoidanceWP.planeID = thisPlane.getID();
	return true;
}

/* Return a container with info used to determine threat danger */
ipn::threatInfo ipn::getThreatInfo(PlaneObject &thisPlane, PlaneObject &otherPlane) {
	double separationDistance, t_go, ZEM;

	Vector2D separation = getSeparationVector(thisPlane, otherPlane);
	Vector2D direction = getDirectionVector(thisPlane, otherPlane);

	if (separation.getX() == 0 && separation.getY() == 0) {
		ROS_INFO("Planes at same position?");
	}
	separationDistance = separation.getMagnitude();
	
	if (separationDistance > 0 && separationDistance < SEPARATION_THRESHOLD) {
		t_go = -1.0 * separation.dot(direction)
			/ (thisPlane.getSpeed() * direction.dot(direction));
		ZEM = sqrt( separation.dot(separation)
			+ 2*thisPlane.getSpeed()*t_go*separation.dot(direction)
			+ pow(thisPlane.getSpeed()*t_go, 2)*direction.dot(direction) );

	} else {
		t_go = std::numeric_limits<double>::max();
		ZEM = std::numeric_limits<double>::max();
	}

/*
	if (separationDistance > 0 && separationDistance < COLLISION_THRESHOLD) {
		ROS_ERROR("Distance between #%d and %d is: %f",
			thisPlane.getID(), otherPlane.getID(), separationDistance);
	}
*/

	threatInfo threat;
	threat.threatPlane = &otherPlane;
	threat.separation = separation;
	threat.direction = direction;
	threat.separationDistance = separationDistance;
	threat.t_go = t_go;
	threat.ZEM = ZEM;

	return threat;
}

/* Return pointer to threat with greatest danger of collision */
/* Returns NULL if none of the threats need avoidance */
ipn::threatInfo ipn::findGreatestThreat(std::vector<threatInfo> &allThreats) {
	int i;
	double t_go, ZEM;
	int size = allThreats.size();
	threatInfo greatestThreat; // = NULL;
	greatestThreat.threatPlane == NULL;

	for (i = 0; i < size; i++) {
		t_go = allThreats[i].t_go;
		ZEM = allThreats[i].ZEM;

		//if (ZEM < 0 || t_go < 0 || ZEM > ZEM_THRESHOLD || t_go > T_GO_THRESHOLD) {
		if (ZEM < 0 || t_go < 0 || ZEM > ZEM_THRESHOLD) {
			continue;
		}

		// if (fabs( allThreats[i].separation.getAngle() ) > 67.5) {
		// 	continue;
		// }

		if (greatestThreat.threatPlane == NULL) {
			greatestThreat = allThreats[i];
		} else if (ZEM <= CONFLICT_THRESHOLD) {
			if (greatestThreat.ZEM <= CONFLICT_THRESHOLD) {
				if (t_go < greatestThreat.t_go || ZEM < greatestThreat.ZEM) {
					greatestThreat = allThreats[i];
				}
			} else {
				greatestThreat = allThreats[i];
			}
		} else if (t_go < greatestThreat.t_go && greatestThreat.ZEM > CONFLICT_THRESHOLD) {
			greatestThreat = allThreats[i];
		}
	}

	return greatestThreat;
}

/*  */
bool ipn::shouldTurnRight(PlaneObject &thisPlane, threatInfo &threat) {
	bool turnRight;

	double thisPlaneBearing = thisPlane.getCurrentBearing();
	double threatPlaneBearing = threat.threatPlane->getCurrentBearing();

	double LOS_angle = threat.separation.getAngle();
	double theta_1 = LOS_angle - thisPlaneBearing;
	double theta_2 = LOS_angle - threatPlaneBearing;

	// double temp = cos((thisPlaneBearing - threatPlaneBearing)*DEGREES_TO_RADIANS);
	// if (fabs(temp) >= 0.5) {
	// 	turnRight = (theta_1 < theta_2);
	// } else {
	// 	turnRight = !(theta_1 < theta_2);
	// }

	turnRight = (sin(theta_2*DEGREES_TO_RADIANS) - sin(theta_1*DEGREES_TO_RADIANS)) >= 0;

	 return turnRight;
}

/*  */
waypoint ipn::createAvoidanceWaypoint(PlaneObject &thisPlane, threatInfo &threat) {
	waypoint wp;
	waypoint currentLoc = thisPlane.getCurrentLoc();
	double currentBearing = thisPlane.getCurrentBearing() * DEGREES_TO_RADIANS;
	double newBearing;
	double turnAngle = MAXIMUM_TURNING_ANGLE * exp(-1.0 * threat.ZEM / ZEM_THRESHOLD);
	if (shouldTurnRight(thisPlane, threat)) {
		turnAngle = -1.0 * turnAngle;
	}
	newBearing = currentBearing + turnAngle * DEGREES_TO_RADIANS;

	wp.latitude = currentLoc.latitude + thisPlane.getSpeed() * sin(newBearing) / threat.separation.getLatToMeters();
	wp.longitude = currentLoc.longitude + thisPlane.getSpeed() * cos(newBearing) / threat.separation.getLonToMeters();
	wp.altitude = currentLoc.altitude;

	return wp;
}
