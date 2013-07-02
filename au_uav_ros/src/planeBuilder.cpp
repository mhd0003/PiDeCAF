#include "au_uav_ros/planeBuilder.h"
using namespace au_uav_ros;

int PlaneBuilder::buildPlane(struct waypoint wp, Coordinator &c) {
	int planeID = findPlaneID(c);
	if (planeID != -1) {
		c.planes[planeID] = PlaneObject(planeID, wp);
	}
	return planeID;
}

int PlaneBuilder::buildSimPlane(struct waypoint wp, Coordinator &c, SimPlane &s) {
	int planeID = findPlaneID(c);
	if (planeID != -1) {
		c.simPlanes[planeID] = SimPlaneObject(planeID, wp);
		s.request.planeID = planeID;
		s.request.clear = false;
		s.request.size = 1;
		s.request.latitudes.push_back(wp.latitude);
		s.request.longitudes.push_back(wp.longitude);
		s.request.altitudes.push_back(wp.altitude);
	}
	return planeID;
}

bool PlaneBuilder::buildCourse(std::string filename, Coordinator &c, std::map<int, SimPlane> &s) {
    //open our file
    FILE *fp;
    fp = fopen(filename.c_str(), "r");

    //check for a good file open
    if(fp != NULL)
    {
        char buffer[256];

        std::map<int, bool> isFirstPoint;
        while(fgets(buffer, sizeof(buffer), fp))
        {
            if(buffer[0] == '#' || isBlankLine(buffer))
            {
                //this line is a comment
                continue;
            }
            else
            {
		//set some invalid defaults
                int planeID = -1;
                struct waypoint temp;
		int normal;
                temp.latitude = temp.longitude = temp.altitude = -1000;

                //parse the string
                sscanf(buffer, "%d %lf %lf %lf %d\n", &planeID, &temp.latitude, &temp.longitude, &temp.altitude, &normal);
                //check for the invalid defaults
                if(planeID == -1 || temp.latitude == -1000 || temp.longitude == -1000 || temp.altitude == -1000)
                {
                    //this means we have a bad file somehow
                    ROS_ERROR("Bad file parse");
                    return false;
                }

                //check our map for an entry, if we dont have one then this is the first time
                //that this plane ID has been referenced so it's true
                if(isFirstPoint.find(planeID) == isFirstPoint.end())
                {
                    if (/*planeID >= 0 && planeID <= 31*/normal == 0) { /*TODO Calculate Sim Plane Differently */
                        isFirstPoint[planeID] = true;
			//planeID = findPlaneID(c); TODO
			if (planeID == -1) {
				ROS_ERROR("Couldn't create plane");
                            	return false;
			}
			//Create New Plane Here
			c.simPlanes[planeID] = SimPlaneObject(planeID, temp);
			s[planeID].request.size = 1;
			s[planeID].request.clear = false;
			s[planeID].request.planeID = planeID;
			s[planeID].request.latitudes.push_back(temp.latitude);
			s[planeID].request.longitudes.push_back(temp.longitude);
			s[planeID].request.altitudes.push_back(temp.altitude);
			/* TODO Make Simulated Planes 
			//this is the first time we've seen this ID in the file, attempt to create it
                        CreateSimulatedPlane srv;
                        srv.request.startingLatitude = temp.latitude;
                        srv.request.startingLongitude = temp.longitude;
                        srv.request.startingAltitude = temp.altitude;
                        srv.request.startingBearing = 0;
                        srv.request.requestedID = planeID;

                        //send the service request
                        printf("\nRequesting to create new simulated plane with ID #%d...\n", planeID);
                        if(createSimulatedPlaneClient.call(srv))
                        {
                            printf("New simulated plane with ID #%d has been created!\n", srv.response.planeID);
                        }
                        else
                        {
                            ROS_ERROR("Did not receive a response from simulator");
                        } */
                    }
                    else if (/*planeID >= 32 && planeID <= 63*/normal == 1){ /*TODO Calculate Real Planes Differently */
                        isFirstPoint[planeID] = true;
			//planeID = findPlaneID(c);
                        if(planeID == -1)
                        {
                            ROS_ERROR("Couldn't create plane with ID %d", planeID);
                            return false;
                        }

			//Create New Plane Here
			c.planes[planeID] = PlaneObject(planeID, temp);
                    }
                    else {
                        //this means we have a bad file somehow
                        ROS_ERROR("Bad file parse");
                        return false;
                    }
                } else {
			isFirstPoint[planeID] = false;
			//TODO get plane ID
			if (/*c.planes.find(planeID) != c.planes.end()*/normal == 1) {
				c.planes[planeID].addWp(temp, true);
			} else {
				c.simPlanes[planeID].addWp(temp, true);
				s[planeID].request.size++;
				s[planeID].request.latitudes.push_back(temp.latitude);
				s[planeID].request.longitudes.push_back(temp.longitude);
				s[planeID].request.altitudes.push_back(temp.altitude);
			}
		}

                //only clear the queue with the first point
                if(isFirstPoint[planeID]) isFirstPoint[planeID] = false;
            }
        }

        //if we make it here everything happened according to plan
        return true;
    }
    else
    {
        ROS_ERROR("Invalid filename or location: %s", filename.c_str());
        return false;
    }

}

int PlaneBuilder::findPlaneID(Coordinator &c) {
	int newId = 2; //planeid of 1 is for new planes who need a planeid

	while(newId < 256) //plane id can't be larger than 255
	{
		//check if the ID is occupied or inactive
		if((c.planes.find(newId) != c.planes.end()) || (c.simPlanes.find(newId) != c.simPlanes.end())) //removed && planesArray[id].isActive
		{
			//this id already exists, increment our id and try again
			newId++;
		}
		else
		{
			//we found an unused ID, lets steal it
			//creation happens later planes[newId] = PlaneObject();
			//removed planesArray[id].isActive = true;
			//numPlanes++;
				
			return  newId;
		}
	}
		
		//plane id is out of bounds
		return -1;
}
