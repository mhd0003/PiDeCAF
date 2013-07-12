#ifndef _IPN_H_
#define _IPN_H_

#include "ros/ros.h"
#include "au_uav_ros/plane.h"
#include "au_uav_ros/vector2D.h"
using au_uav_ros::Vector2D;

namespace ipn {
	static double SEPARATION_THRESHOLD = 0.0;
	static double ZEM_THRESHOLD = 0.0;
	const static double T_GO_THRESHOLD = 10.0;
	const static double MU = 50.0;


	struct threatInfo {
		Plane *threatPlane;
		Vector2D separationV, directionV;
		double separationDistance, t_go, ZEM;
	};

	Vector2D getSeparationVector(const Plane &plane1, const Plane &plane2);
	Vector2D getDirectionVector(const Plane &plane1, const Plane &plane2);

	bool checkForThreats(Plane &thisPlane, std::map<int, Plane> &planeMap, waypoint &avoidanceWP);
	threatInfo getThreatInfo(Plane &thisPlane, Plane &otherPlane);
	threatInfo* findGreatestThreat(std::vector<threatInfo> &allThreats);
	bool shouldTurnRight(Plane &thisPlane, threatInfo &threat);
	waypoint createAvoidanceWaypoint(Plane &thisPlane, threatInfo &threat);
};

#endif
