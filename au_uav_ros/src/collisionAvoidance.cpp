#include "au_uav_ros/collisionAvoidance.h"
using namespace au_uav_ros;

void CollisionAvoidance::avoid(int id, std::map<int, au_uav_ros::PlaneObject> planes, std::map<int, au_uav_ros::SimPlaneObject> simPlanes, std::vector<waypoint> &wps) {
	std::map<int, au_uav_ros::PlaneObject> allPlanes;
	allPlanes.insert(planes.begin(), planes.end());
	allPlanes.insert(simPlanes.begin(), simPlanes.end());

	waypointContainer bothNewWaypoints = findNewWaypoint(allPlanes[id], planes);
	waypoint newWaypoint = bothNewWaypoints.plane1WP;
	
	if (bothNewWaypoints.plane2ID >= 0) {
		waypoint newWaypoint2 = bothNewWaypoints.plane2WP;
		newWaypoint2.planeID = bothNewWaypoints.plane2ID;
		wps.push_back(newWaypoint2);

		newWaypoint.planeID = id;
		wps.push_back(newWaypoint);
		
	} /* TODO THIS IS BROKE */
	/* TODO if ((requestWaypointInfoSrv.response.longitude == newWaypoint.longitude) 
		&& (requestWaypointInfoSrv.response.latitude == newWaypoint.latitude)) {return;} what does this do */
	/**newWaypoint.planeID = id;
	wps.push_back(newWaypoint); */
}
