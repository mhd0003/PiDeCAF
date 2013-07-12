
#ifndef _PLANE_H_
#define _PLANE_H_

#include "au_uav_ros/standardDefs.h"
#include "au_uav_ros/Telemetry.h"
#include "au_uav_ros/vector2D.h"
using au_uav_ros::Telemetry;
using au_uav_ros::waypoint;

namespace ipn {
	const waypoint INVALID_GPS_WP;

	class Plane {
	public:
		Plane();
		Plane(const Telemetry &update);

		int getID() const;
		double getCurrentBearing() const;
		double getGroundSpeed() const;
		waypoint getCurrentLocation() const;
		waypoint getAvoidanceWaypoint() const;
		waypoint getGoalWaypoint() const;
		waypoint getDestinationWaypoint() const;
		std::map<int, Plane> getPlaneMap();

		void setID(int idIn);
		void setAvoidanceWaypoint(const waypoint &wpIn);
		void setGoalWaypoint(const waypoint &wpIn);
		void update(const Telemetry &update);

	private:
		int planeID;
		double currentBearing;
		double groundSpeed;

		waypoint currentLocation;
		Telemetry lastUpdate;

		waypoint avoidWP;
		waypoint goalWP;
		std::map<int, Plane> planeMap;

		void updateThisPlane(const Telemetry &update);
		void updatePlaneMap(const Telemetry &update);
		void updatePosition(const Telemetry &update);
	};

};

#endif