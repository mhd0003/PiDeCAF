#ifndef _COLLISION_AVOIDANCE_H_
#define _COLLISION_AVOIDANCE_H_

#include <vector>

#include "au_uav_ros/Telemetry.h"

#include "au_uav_ros/planeObject.h"
#include "au_uav_ros/ripna.h"
#include "au_uav_ros/simPlaneObject.h"
#include "au_uav_ros/standardFuncs.h"

namespace au_uav_ros {
	class CollisionAvoidance {
	private:
		
	public:
		virtual void avoid(int id, 
				std::map<int, au_uav_ros::PlaneObject> planes, 
				std::map<int, au_uav_ros::SimPlaneObject> simPlanes,
				std::vector<waypoint> &wps);
	};
}
#endif
