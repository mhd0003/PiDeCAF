/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This file contains an implementation of the ForceField object that is required for the F^2 approach
		(ADD MORE)

Date: 6/13/13
*/


#include "au_uav_ros/ForceField.h"

/**************************************************************************
 * 								FORCE FIELD								  *
 **************************************************************************
 The following methods are members of the ForceField object which carries
 a FieldShape and a FieldFunction */



/* Default constructor, initializes a ForceField with an oval shape and a
 * bivariate normal function
 *
 * TODO:
 * 		Create a constructor that enables choosing a field and a shape
 */
ForceField::ForceField(){
	myShape = new OvalField();
	myFunction = new BivariateNormal();
}

/*
ForceField::~ForceField(){
	delete myShape;
	delete myFunction;
}
*/

const FieldShape * ForceField::getMyShape() const{
	return myShape;
}

const FieldFunction * ForceField::getMyFunction() const{
	return myFunction;
}

ForceField& ForceField::operator=(const ForceField& ForceFieldIn){
	*myShape = *(ForceFieldIn.getMyShape());
	*myFunction = *(ForceFieldIn.getMyFunction());

	return *this;
}


/* areCoordinatesInMyField(...)
 * Description:
 * 		Calls the ForceField's FieldShape member to determine whether or not
 * 		a relative coordinate lies within this ForceField's zone of influence
 */
bool ForceField::areCoordinatesInMyField(fsquared::relativeCoordinates positionInField, double fieldAngle, double planeAngle){
	return myShape->internal_areCoordinatesInMyField(positionInField, fieldAngle, planeAngle);
}

/* findForceMagnitude(...)
 * Description:
 * 		Calls the ForceField's FieldFunction member to determine the force exerted
 * 		by the ForceField on a set of coordinates.
 * 	Assumptions:
 * 		Coordinates are within the field shape
 * 	TODO: RENAME TO INTERNAL
 */
double ForceField::findForceMagnitude(fsquared::relativeCoordinates positionInField){
		return myFunction->findFieldFunctionMagnitude(positionInField);
}



/**************************************************************************
 * 								SHAPES									  *
 **************************************************************************/


/*	Constructor for an oval field
 * 	TODO:
 * 		make another constructor that allows choosing constants
 *
 */
OvalField::OvalField(){
	shapeParams.gamma = 1500;
	shapeParams.alphaTop = .5;
	shapeParams.alphaBot = .5;
	shapeParams.betaTop = .25;
	shapeParams.betaBot = 1.6;
}

//internal_areCoordinatesInMyField(...)
//Precondition: None
//Use:
//		This method will determine whether or not coordinates are inside
//		this field
//Params:
//		positionInField: coordinate of the plane that will feel the force
//						 relative to the position of the plane generating the field
//		fieldAngle: bearing of the plane generating the field - angle between the plane
//					genereating the field and the plane that will feel the force
//		planeAngle:	angle between the two planes, calculated starting from the generating
//					plane and going to the plane that will feel the force
//TODO REMOVE ANGLES
bool OvalField::internal_areCoordinatesInMyField(fsquared::relativeCoordinates positionInField, double fieldAngle, double planeAngle){
	int x = positionInField.x;
	int y = positionInField.y;
	if (y < 0){
		// plane feeling the force is behind the plane exerting the force, therefore use the bottom boundary
		double forceLimit = -sqrt((shapeParams.gamma-(shapeParams.alphaBot*pow(x,2)))/shapeParams.betaBot);
		if (y>forceLimit) return true;
		else return false;
	}
	else{
		// plane feeling the force is in front, therefore use the top boundary
		double forceLimit = sqrt((shapeParams.gamma-(shapeParams.alphaTop*pow(x,2)))/shapeParams.betaTop);
		if (y<forceLimit) return true;
		else return false;
	}
}


/**************************************************************************
 * 								Functions								  *
 **************************************************************************

/* Constructor for bivariate normal function
 * TODO:
 * 		make another constructor that allows choosing constants
 */
BivariateNormal::BivariateNormal(){
	functionParams.maxForce = 4000;
	functionParams.alpha = .00129;
	functionParams.beta = .000850;
}

double BivariateNormal::findFieldFunctionMagnitude(fsquared::relativeCoordinates positionInField){
	int x = positionInField.x;
	int y = positionInField.y;
	return functionParams.maxForce * exp(-functionParams.alpha*pow(x,2)-functionParams.beta*pow(y,2));
}


