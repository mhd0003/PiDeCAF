#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include "au_uav_ros/Telemetry.h"
#include "au_uav_ros/Command.h"

namespace au_uav_ros	{
	class CollisionAvoidance	{
	private:
	public:
		void init();	
		au_uav_ros::Command avoid(au_uav_ros::Telemetry telem);	//Called when there's a telemetry callback.
	};
}

#endif
