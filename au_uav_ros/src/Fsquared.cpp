/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This is an implementation of the fsquared algorithm for collision avoidance and detection.
		For a description of each of these functions refer to Fsquared.h, comments in this file
		will be limited to comments on implementation


Date: 6/13/13


*/
#include <ros/console.h>
#include "au_uav_ros/Fsquared.h"
#include "au_uav_ros/planeObject.h"

//defines from 2012 APF group to resolve looping
#define MAXIMUM_TURNING_ANGLE 22.5 //degrees
#define LOOPING_DISTANCE 4*MPS_SPEED		// When distance to destination is less than this distance, we should start checking for looping
#define LOOP_RADIUS 28.64058013			// Turning radius of the UAV = MPS_SPEED/sin(MAXIMUM_TURNING_ANGLE)*sin((180-MAXIMUM_TURNING_ANGLE)/2)
						// Because the simulator assumes straight turns, we used the law of sines to find the distance from the
						// points on the perimeter of the hexadecagon generated by a looping UAV to the center of hexadecagon

/*
Credit: 2012 APF group
Calculates the distance between the center of "circle" generated when a plane is looping around its destination
and the plane's destination.  The angle parameter is the number of degrees to the left or right of the UAV the center of
turning is located.  The angle parameter is based off of the turning angle per second.  Because the simulator
uses straight line paths between each of the waypoints given, a looping UAV actually creates a hexadecagon.
For example, in this code 22.5 is our turning angle. Therefore, the angle provided for this method is either 101.25 or -101.25 since
[(180-22.5)/2 = 78.75; 78.75 + 22.5 = 101.25].
*/
double findLoopDistance(const au_uav_ros::PlaneObject &pobj1, double angle){
	//actual bearing of plane in cartesian coordinates
	double UAVTheta = pobj1.getCurrentBearing();
	//Angle from bearing of plane to center of turning radius circle in cardinal coordinates.
	double bearingToCenter = manipulateAngle(UAVTheta+angle);

	//angular distance to center of the "circle" generated by a looping UAV
	double distToCenter = LOOP_RADIUS/EARTH_RADIUS;

	//current position of plane
	au_uav_ros::waypoint position;
	position.latitude = pobj1.getCurrentLoc().latitude;
	position.longitude = pobj1.getCurrentLoc().longitude;
	position.altitude = pobj1.getCurrentLoc().altitude;

	//calculate location of the center of the "circle" generated by a looping UAV
	au_uav_ros::waypoint centerPosition = calculateCoordinate(position, bearingToCenter, distToCenter);

	//distance between center and destination
	double centerToWaypoint = findDistance(centerPosition.latitude, centerPosition.longitude,
					pobj1.getDestination().latitude, pobj1.getDestination().longitude);

	return centerToWaypoint;
}

/*
Credit: Copied from 2012 APF group
Method determines if aircraft is making a maximum angle turn and is within looping distance of its destination.
If so, it finds the distance between the destination of the UAV and the center of the circle made by a looping UAV,
and determines if the destination is unreachable.
*/
bool inLoop(const au_uav_ros::PlaneObject &pobj1, au_uav_ros::mathVector &tForce){
	bool inLoop = false;
	//angle of actual UAV bearing in degrees w/ respect to planeObject coordinates
	double UAVTheta = manipulateAngle(pobj1.getCurrentBearing() + 180);
	double distanceToDest = findDistance(pobj1.getCurrentLoc().latitude, pobj1.getCurrentLoc().longitude, pobj1.getDestination().latitude, pobj1.getDestination().longitude);

	//Find angle between force vector and UAV
	double deltaTheta = tForce.getDirection() - UAVTheta;

	//Make sure angle is on interval [-180, 180]
	deltaTheta = manipulateAngle(deltaTheta);

	//Check to see if attempting a maximum turn
	if (deltaTheta > MAXIMUM_TURNING_ANGLE){
		//Check for looping
		if (distanceToDest < LOOPING_DISTANCE){
			// find distance between center of "circle" made when the UAV is looping and the destination
			double centerToWaypoint = findLoopDistance(pobj1,101.25);

			//if the centerToWaypoint distance is less than LOOP_RADIUS-COLLISION_THRESHOLD, destination will not be reached
			if(centerToWaypoint < (LOOP_RADIUS-COLLISION_THRESHOLD)){
				inLoop = true;
			}else{
				inLoop = false;
			}
		}
	}

	//Check to see if attempting a maximum turn
 	 if(deltaTheta < -MAXIMUM_TURNING_ANGLE){
		//Check for looping
		if (distanceToDest < LOOPING_DISTANCE){
			// find distance between center of "circle" made when the UAV is looping and the destination
			double centerToWaypoint = findLoopDistance(pobj1,-101.25);

			//if the centerToWaypoint distance is less than LOOP_RADIUS-COLLISION_THRESHOLD, destination will not be reached
			if(centerToWaypoint < (LOOP_RADIUS-COLLISION_THRESHOLD)){
				inLoop = true;
			}else{
				inLoop = false;
			}
		}
	}
	return inLoop;
}


au_uav_ros::waypoint fsquared::findTempForceWaypoint(au_uav_ros::PlaneObject &me, const au_uav_ros::Telemetry &msg){

	//If the telemetry update is not from "me", update "me's"
	//map of other planes that are exerting a force on "me"
	if(me.getID() != msg.planeID){

		//create enemy plane, 12 is the collision radius
		au_uav_ros::PlaneObject enemy(12, msg);

		//check to see if the updated plane is within RADAR_ZONE
		if(me.findDistance(enemy) > RADAR_ZONE){
			//plane is out of the RADAR_ZONE
			//take enemy out of the map if it is in the map
			me.planeOut_updateMap(enemy);
		}

		else{
			//plane is in the RADAR_ZONE
			if(inEnemyField(me, enemy)){
				//enemy is exerting a force on "me"
				me.planeIn_updateMap(enemy);
			}
			else{
				//enemy is not exerting a force on "me"
				me.planeOut_updateMap(enemy);
			}
		}
	}

	//msg is from "me", need to update "me"
	else{
		me.setCurrentLoc(msg.currentLatitude, msg.currentLongitude, msg.currentAltitude);
		me.setCurrentBearing(msg.targetBearing);
	}

	//calculate next direction to travel in
	au_uav_ros::mathVector resultantForce(0,0), attractiveForce(0,0), repulsiveForce(0,0);

	//commented out to test navigation without APF avoidance
	repulsiveForce = fsquared::sumRepulsiveForces(me, me.getMap());
	attractiveForce = fsquared::calculateAttractiveForce(me, me.getDestination());

	resultantForce = repulsiveForce + attractiveForce;

	//DEBUG
	//ROS_ERROR("Plane %d has attractiveForce of magnitude %f and direction %f", me.getID(),attractiveForce.getMagnitude(), attractiveForce.getDirection());
	ROS_ERROR("Plane %d repulsive:mag:and dir (%f|%f)", me.getID(), repulsiveForce.getMagnitude(), repulsiveForce.getDirection());
	//ROS_ERROR("Plane %d has resultantForce of magnitude %f and direction %f", me.getID(), resultantForce.getMagnitude(), resultantForce.getDirection());
	au_uav_ros::coordinate meCurrentCoordinates = me.getCurrentLoc();	//latitude and longitude defining where me is now
	au_uav_ros::waypoint meCurrentWaypoint;			//holds same information as meCurrentCoordinates, but will
													//be formatted as a waypoint with altitude 0


	meCurrentWaypoint.latitude = meCurrentCoordinates.latitude;
	meCurrentWaypoint.longitude = meCurrentCoordinates.longitude;
	meCurrentWaypoint.altitude = 0;

	/*
	if (inLoop(me, resultantForce)){
		//pobj1 is in a loop, modify attractive force so that the destination is actually repulsive to break the cycle
		attractiveForce.setDirection(manipulateAngle(attractiveForce.getDirection() + 180));

		//we changed aForce so now need to recalculate tForce
		resultantForce = repulsiveForce + attractiveForce;
	}
	*/
	return fsquared::motionVectorToWaypoint(resultantForce.getDirection(), meCurrentWaypoint, (WP_GEN_SCALAR));
}

//-----------------------------------------
//Fields
//-----------------------------------------



/*
 * findFieldAngle
 * Preconditions:	assumes valid planes
 * params:		me: plane that is potentially in enemy's field
 * 			enemy: plane that is producing field
 * returns:		field angle - angle between enemy's bearing and my location.
 * 				0  < x < 180  = to enemy's right
 * 				-180< x < 0= to enemy's left 
 *
 * note: Different from au_uav_ros::PlaneObject::findAngle(). FindAngle finds
 * the angle between the relative position vector from one plane to another and the
 * x axis in a global, absolute x/y coordinate system based on latitude and
 * longitude.
 */
double fsquared::findFieldAngle(au_uav_ros::PlaneObject& me, au_uav_ros::PlaneObject &enemy)	{

	//Make two vectors - one representing bearing of enemy plane
	//		     the other representing relative position from
	//		     enemy to me
	au_uav_ros::mathVector enemyBearing(1, toCartesian(enemy.getCurrentBearing()));
	au_uav_ros::mathVector positionToMe(1, enemy.findAngle(me));


	//Find angle between two vectors
	return enemyBearing.findAngleBetween(positionToMe); 
}

/*
 * precondition: me and map are not null
 * 		 map only contains planes  exerting a repulsive force on me
 */
au_uav_ros::mathVector fsquared::sumRepulsiveForces(au_uav_ros::PlaneObject &me, std::map<int, au_uav_ros::PlaneObject> & planesToAvoid)	{
	
	au_uav_ros::mathVector sum, current;

	std::map<int, au_uav_ros::PlaneObject> :: iterator it;
	for(it = planesToAvoid.begin(); it!= planesToAvoid.end(); it++)	{
		ROS_INFO("The plane in my map has coordinates: %f,%f and bearing %f", it-> second.getCurrentLoc().latitude,
			it-> second.getCurrentLoc().longitude, it-> second.getCurrentBearing());
		current = calculateRepulsiveForce(me, it-> second);
		sum += current;
	}
	return sum;
}


/* rightHandTurnRule(...)
 * Description:
 * 		Helper function to calculateRepulsiveForce
 * 		Determines if a plane should be forced to make a right turn to travel behind a plane approaching from the right.
 * 		Returns a modified value of rAngle if the conditions favor a forced turn
 * 	Credit:
 * 		Code from 2012 APF group
 */

double rightHandTurnRule(double fieldAngle, double rAngle, double distance, au_uav_ros::PlaneObject &me){
	//Calculates angle between pobj1's bearing and the direction of repulsion force.
	double rToBearing = me.getCurrentBearing() - rAngle;
	ROS_INFO("Plane %d, my field angle is %f", me.getID(), fieldAngle);
	//aAngle is the angle between the bearing of me and the location of its destination
	double aAngle = findAngle(me.getCurrentLoc().latitude, me.getCurrentLoc().longitude, me.getDestination().latitude, me.getDestination().longitude);
	//adjust fieldAngle to match function specifications
	fieldAngle = manipulateAngle(fieldAngle);
	if(fieldAngle < 0 && fieldAngle > -135 && rToBearing < -90 && rToBearing > -180){
		double a = 0.0, b = 0.0, c = 0.0, A = 0.0, B = 0.0, C = 0.0;

		//Spherical law of cosines calculations

		if(fieldAngle > -25){
			a = distance / EARTH_RADIUS;
			B = -1 * fieldAngle * PI / 180;
			C = (180 - (-1 * rToBearing)) * PI / 180;

			A = acos(-cos(B)*cos(C) + sin(B)*sin(C)*cos(a));

			b = acos((cos(B) + cos(C)*cos(A))/(sin(C)*sin(A)));
			b *= EARTH_RADIUS;

			c = acos((cos(C) + cos(A)*cos(B))/(sin(A)*sin(B)));
			c *= EARTH_RADIUS;
		}

		//Angle between bearing and destination
		double aToBearing = me.getCurrentBearing() - aAngle;

		if((c - b) > ((-1 * rToBearing) - 90.0) && fieldAngle > -25){
			//Plane should not turn right, because it will turn into a plane
			return rAngle;
		}else if(aToBearing < 0 && fieldAngle < -90){
			//Plane should not turn right, parallel to other plane and its destination is to the left
			return rAngle;
		}else{
			//flip repulsive force across bearing to force right turn.
			ROS_INFO("***********************RIGHT HAND RULE TAKING EFFECT***************");
			return manipulateAngle(2 * (180 + rToBearing) + rAngle);
		}
	}

	return rAngle;

}




/* Assumptions:
 * 		Only calculates radially repuslive forces from enemy to "me"
 *
 * Pseudocode:
 * 		find "me" coordinates from enemy's POV
 *		check to see if these coordinates are within enemy's field
 *		if "me" is in the enemys field
 *			find the repulisve force
 *		else
 *			the repulsive force imparted by enemy has magnitude 0
 *
 * Variables:
 * 		fieldAngle: angle between the bearing of the plane generating the force to the location
 *		rMagnitude: magnitude of the repuslive force vector
 *		rAngle:		angle of repulsive force
 *		relativePosition: x and y difference in position between me and enemy from
 *						  enemy's POV
 *
 * TODO:
 * 		Add a feel function
 * 		Incorporate "right hand turn" rules
 */

au_uav_ros::mathVector fsquared::calculateRepulsiveForce(au_uav_ros::PlaneObject &me, au_uav_ros::PlaneObject &enemy){
   	double fieldAngle, planeAngle, rMagnitude, rAngle;
	fsquared::relativeCoordinates relativePosition;
	bool insideEnemyField;


	fieldAngle = fsquared::findFieldAngle(me, enemy);
	//find the angle from enemy's position to "me"s position
	planeAngle = enemy.findAngle(me);
	//find "me" coordinates from enemy's POV
	relativePosition = fsquared::findRelativePosition(me, enemy);
	//determine whether or not "me" is in enemy's field
	insideEnemyField = fsquared::inEnemyField(enemy, relativePosition, fieldAngle, planeAngle);
	//if "me" is in enemy field
	if(insideEnemyField){
		//calculate the force exerted by the field on "me"
		rMagnitude = enemy.getField().findForceMagnitude(relativePosition);
		//calculate the angle of the force exerted by the field onto me
		rAngle = planeAngle; //changed from toCartesian(planeAngle - 180) to planeAngle
		//check to see if "me" should try to pass behind another plane
//		rAngle = rightHandTurnRule(fieldAngle, rAngle, me.findDistance(enemy), me);
		au_uav_ros::mathVector repulsiveForceVector(rMagnitude, rAngle);
		return repulsiveForceVector;
	}
	//"me" is not in the enemy's field, return a vector with 0 magnitude (no contribution to force)
	else{
		au_uav_ros::mathVector repulsiveForceVector(0,0);
		return repulsiveForceVector;
	}
}

/* Assumptions:
 * 		The magnitude of the attractive force to the waypoint is defined correctly
 *
 * Pseudocode:
 * 		Find angle to waypoint
 * 		Set magnitude of attractive force
 * 		Return force vector
 *
 * Credit:
 * 		Derived from 2012 APF Group
 *
 */

au_uav_ros::mathVector fsquared::calculateAttractiveForce(au_uav_ros::PlaneObject &me, au_uav_ros::waypoint goal_wp){
	double aAngle, aMagnitude, destLat, destLon, currentLat, currentLon;
	//obtain current location by accessing PlaneObject's current coordinate
	currentLat = me.getCurrentLoc().latitude;
	currentLon = me.getCurrentLoc().longitude;
	destLat = goal_wp.latitude;
	destLon = goal_wp.longitude;
	aAngle = findAngle(currentLat, currentLon, destLat, destLon);
	aMagnitude = ATTRACTIVE_FORCE;
	//construct the attractive force vector and return it
	au_uav_ros::mathVector attractiveForceVector(aMagnitude, aAngle);
	return attractiveForceVector;
}


/*
 *Precondition: Assume valid planes
 *Use: Find "me's" position from enemy's POV
 *Params:
 *		me: Plane that is potentially in enemy's field
 *		enemy: Plane that is producing the field
 *Returns:	relativeCoordinates in meters of "me" from the enemy's POV, where enemy's bearing is towards the positive y axis.
 *Implementation:
 *			
 *who:		vw
*/
fsquared::relativeCoordinates fsquared::findRelativePosition(au_uav_ros::PlaneObject &me, au_uav_ros::PlaneObject &enemy ){
	fsquared::relativeCoordinates loc;

	double distance = enemy.findDistance(me);
	double fieldAngle = fsquared::findFieldAngle(me, enemy);

	//Find Y axis coordinate (in front or behind enemey)
	loc.y = cos(fieldAngle*PI/180.0)*distance;

	//Find X Axis coordinate (to the left or right)
	loc.x = sin(fieldAngle*PI/180.0)*distance;

	return loc;
}

/* Assumptions:
 * 		Enemy plane has a properly initialized field
 * 	Description:
 * 		All calculations are handeled by the ForceField class, so this function
 * 		retrieves the ForceField associated with a plane and then calls the
 * 		appropriate method to determine whether or not a point is in a specific
 * 		field
 */
bool fsquared::inEnemyField(au_uav_ros::PlaneObject &enemy, fsquared::relativeCoordinates locationOfMe, double fieldAngle, double planeAngle){
	ForceField  enemyField = enemy.getField();
	return enemyField.areCoordinatesInMyField(locationOfMe, fieldAngle, planeAngle);
}

//Overloaded function, performs same action with different input
bool fsquared::inEnemyField(au_uav_ros::PlaneObject &me, au_uav_ros::PlaneObject &enemy){
	double fieldAngle, planeAngle;
	relativeCoordinates relativePosition;

	fieldAngle = findFieldAngle(me, enemy);
	planeAngle = enemy.findAngle(me);
	relativePosition = findRelativePosition(me, enemy);
	ForceField  enemyField = enemy.getField();
	return enemyField.areCoordinatesInMyField(relativePosition, fieldAngle, planeAngle);
}


//-----------------------------------------
//Waypoint Generation
//-----------------------------------------

/*
 *Precondition: Valid waypoint for me_loc 
 *Use: Converts from desired angle heading to a waypoint. Distance to generated waypoint dependent on previously defined scalar WP_GEN_SCALAR. 
 *Params:
 *		motionAngle: angle between [0,360), CCW from positive x axis (longitude axis)
 *		me_coor: "me's" current location 
 *todo:		vw
 */
au_uav_ros::waypoint fsquared::motionVectorToWaypoint(double angle, au_uav_ros::waypoint me_loc, double scalar) {
	au_uav_ros::waypoint dest_wp;

	//Find relative offset for new waypoint.
	double x_delta_meters = scalar*cos(angle*PI/180.0); 
	double y_delta_meters = scalar*sin(angle*PI/180.0); 

	//Calculate new waypoint 
	double dest_wp_long= me_loc.longitude+ (x_delta_meters*METERS_TO_DELTA_LON);
	double dest_wp_lat= me_loc.latitude+ (y_delta_meters*METERS_TO_DELTA_LAT);
	dest_wp.longitude = dest_wp_long;
	dest_wp.latitude = dest_wp_lat;	
	dest_wp.altitude = me_loc.altitude;
	return dest_wp;	
}

