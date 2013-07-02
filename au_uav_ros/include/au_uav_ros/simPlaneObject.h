#ifndef _SIM_PLANE_OBJECT_
#define _SIM_PLANE_OBJECT_

#include "au_uav_ros/planeObject.h"
#include "au_uav_ros/standardDefs.h"

#include "au_uav_ros/Command.h"
#include "au_uav_ros/Telemetry.h"
using namespace au_uav_ros;

namespace au_uav_ros {
	class SimPlaneObject : public PlaneObject {
	protected:
		int updateIndex;
		double simSpeed;
		au_uav_ros::Command lastCommand;
		long long int currentWaypointIndex;
		double distanceToDestination;
		std::map<int, PlaneObject> otherPlanes; //use this to simulate distributed alogrithms
	public:
		SimPlaneObject(void);
		SimPlaneObject(int _id, struct waypoint wp);
		void setSimSpeed(double _simSpeed);
		virtual bool simulate(double duration, au_uav_ros::Telemetry *telem);
		virtual bool handleNewCommand(au_uav_ros::Command newCommand);
		virtual bool fillTelemetryUpdate(double duration, au_uav_ros::Telemetry *update);
	};
}
#endif
