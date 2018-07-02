#include "Arduino.h"
#include "ROBOT.h"
#include "encoder.h"
#include <BasicLinearAlgebra.h>
#include <Geometry.h>
#include "SPI.h"

ROBOT::ROBOT(int encPin8, int encPin9, int encPin10)
{
	  encoder enc8(encPin8),enc9(encPin9),enc10(encPin10); // Instance of the encoder class
	// Initialize encoders and zero them
		enc8.init();enc9.init(); enc10.init();
		enc8.clearCount(); enc9.clearCount(); enc10.clearCount();
}


Rotation ROBOT::ROTATION_MATRIX(float ROT_X,float ROT_Y,float ROT_Z)
{
	//Computes the rotation matrix for each of the joints based on the endocder measurements. Referenced page 132 of notebook.
    	Transformation ROT;
	//Compute the matrix
		ROT.R.FromEulerAngles(ROT_X,ROT_Y,ROT_Z);
		return ROT.R;
}

Point ROBOT::FORWARD_KINEMATICS(Transformation H01,Transformation H12, Transformation H23, Point P3)
{
	//Computes the forward kinematics to find the POI with respect to the base coordinate system
	    Point result;
	    result=(H01.p+H01.R*(H12.p+H12.R*(H23.p+H23.R*P3)));
	    return result;
}

Point ROBOT::CS_ORIGINS(float x, float y, float z)
{
	// Builds the "point" object from coordinates
    	Point result;
  	// Coordinates of point
	    result.X()=x;
	    result.Y()=y;
	    result.Z()=z;
  	// Define transformation matricees
    	return result;
}

void ROBOT::PRINT_ANGLE(float THETA_1,float THETA_2,float THETA_3){
	  //Prints the angle of each of the encoders in radians
	    Serial.print("Encoder 8, 9, and 10 angles (Rad) = ");
	    Serial.print(THETA_1); Serial.print(", ");
	    Serial.print(THETA_2); Serial.print(", ");
	    Serial.print(THETA_3); Serial.print(' ');
}