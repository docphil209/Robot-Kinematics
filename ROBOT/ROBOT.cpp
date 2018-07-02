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

Point ROBOT::Z_HAT_GROUND(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point GROUND_NORMAL){
  //Unit vector normal to the ground plane with respect to the body fixed coordinate system
    // Normalize this vector
      Point Z_HAT_GROUND=GROUND_NORMAL/GROUND_NORMAL.Magnitude(); //Normal to ground plane
      return Z_HAT_GROUND;
}

Point ROBOT::GROUND_NORMAL(Point A_MEAS,Point B_MEAS, Point C_MEAS){
  //Unit vector normal to the ground plane with respect to the body fixed coordinate system
    // Normal vector to the plane of the ground is the corss of vector AB and AC
      Point AB=A_MEAS-B_MEAS;
      Point AC=A_MEAS-C_MEAS;
      Point GROUND_NORMAL=AC.CrossProduct(AB);
      return GROUND_NORMAL;
}

Point ROBOT::D(Point A_MEAS, Point GROUND_NORMAL, Point D_MEAS){
    //I will solve for D based on the MEASured direction of the vector D_MEAS. This is so that the point I use for D is sure to be in the previously defined plane. (Page 144 of notebook)
      float e=A_MEAS.DotProduct(GROUND_NORMAL)/D_MEAS.DotProduct(GROUND_NORMAL); //What to scale vector D_MEAS by so it intersects with the previous plane
      Point D=D_MEAS*e;
}
      
Transformation ROBOT::H_A_0(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point D_MEAS, Point GROUND_NORMAL,Point Z_HAT_GROUND){
  // Calculate the HTM between the body fixed coordinate system and the coordinate system fixed to the inertial reference frame at the contact point
    // Define the coordinate system attached to the ground that has one of it's unit vectors normal to the ground plane
      //Define the coordiante system by defining the unit vectors and then defining the rotation matrix between it and the body fixed frame. Then make the homogeneous transformation matrix with the coordinates of the origin of A wrt O.
        Point TEMP=B_MEAS-A_MEAS; //Allows me to use the Magnitude method
        Point Y_HAT_GROUND=(B_MEAS-A_MEAS)/TEMP.Magnitude(); //Points from A to B
        Point X_HAT_GROUND=Y_HAT_GROUND.CrossProduct(Z_HAT_GROUND);//Normal to Y_HAT_GROUND and Z_HAT_GROUND
        Transformation H_A_O;
        H_A_O.R=X_HAT_GROUND || Y_HAT_GROUND || Z_HAT_GROUND; //Orientation of the frame at A wrt the body fixed frame
        H_A_O.p=-A_MEAS; //Homogeneous transformation matrix
        return H_A_O;    
}

Point ROBOT::CENTER_OF_MASS(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point D_MEAS,Point Z_HAT_GROUND,Transformation H_A_O){
  // Calculate the position of the center of mass with respect to a coordinate system fixed at P0
    // Project the origin of O onto the plane. This projection is the projected position of the CM (E)
        Point A_FRAME_A=H_A_O.p; //Express the vector [0,0,0] (in frame O) in the frame A
        Point E=A_FRAME_A-Z_HAT_GROUND*A_FRAME_A.DotProduct(Z_HAT_GROUND); //CM projected onto the ground plane
}

BLA::Matrix<3,4> ROBOT::CONCATENTATED_GROUND_CONTACT_POINTS(Point B_MEAS, Point C_MEAS, Point D, Transformation H_A_O){
  // Gives ground contact point wrt the CS attached to the ground. (Project all contact points onto the ground plane) THe matrix is the vectors [A,B,C,D]
    BLA::Matrix<3,1> A= {0,0,0}; //vector A wrt the CS attached to the ground
    BLA::Matrix<3,1> B= H_A_O.R*B_MEAS+H_A_O.p; //vector B wrt the CS attached to the ground
    BLA::Matrix<3,1> C= H_A_O.R*C_MEAS+H_A_O.p; //vector C wrt the CS attached to the ground
    BLA::Matrix<3,1> D_TEMP= H_A_O.R*D+H_A_O.p; //vector D wrt the CS attached to the ground
    BLA::Matrix<3,4> GCP=A||B||C||D_TEMP;
}
