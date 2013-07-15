#ifndef _IPN_H_
#define _IPN_H_

#include "ros/ros.h"
#include "au_uav_ros/planeObject.h"
#include "au_uav_ros/vector2D.h"
using au_uav_ros::PlaneObject;
using au_uav_ros::Vector2D;
using au_uav_ros::waypoint;

namespace ipn {
	static double SEPARATION_THRESHOLD = 0.0;
	static double ZEM_THRESHOLD = 0.0;
	const static double T_GO_THRESHOLD = 10.0;
	const static double MU = 50.0;


	struct threatInfo {
		PlaneObject *threatPlane;
		Vector2D separation, direction;
		double separationDistance, t_go, ZEM;
	};

	Vector2D getSeparationVector(const PlaneObject &plane1, const PlaneObject &plane2);
	Vector2D getDirectionVector(const PlaneObject &plane1, const PlaneObject &plane2);

	bool checkForThreats(PlaneObject &thisPlane, std::map<int, PlaneObject> &planeMap, waypoint &avoidanceWP);
	threatInfo getThreatInfo(PlaneObject &thisPlane, PlaneObject &otherPlane);
	threatInfo findGreatestThreat(std::vector<threatInfo> &allThreats);
	bool shouldTurnRight(PlaneObject &thisPlane, threatInfo &threat);
	waypoint createAvoidanceWaypoint(PlaneObject &thisPlane, threatInfo &threat);
};

#endif
