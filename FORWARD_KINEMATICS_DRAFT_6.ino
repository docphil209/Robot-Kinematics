// Here I confirm that forward kinematics works on the Arduino. This formulation of coordinate systems is less messy.


//Library for robot kinematics
  #include <ROBOT.h>

  
//Libraries for linear algebra and 3D body representations on Arduino
  #include <BasicLinearAlgebra.h>
  #include <Geometry.h>

//Libraries for encoder and buffer communication
  #include "encoder.h"
  #include "SPI.h"
  
// Use the ROBOT library
  ROBOT leftarm(8,9,10);
  
//Encoder related definitions
  int encPin8 = 8, encPin9 = 9, encPin10 = 10; // Encoder pin numbers
  encoder enc8(encPin8),enc9(encPin9),enc10(encPin10); // Instance of the encoder class
  float ENC10_ANGLE_RAD, ENC9_ANGLE_RAD, ENC8_ANGLE_RAD; // Encoder angle (Radians)

// Geometric parameters for right shoulder
  float lrs1=43.87, lrs2=178.8, lrs3=221.2; //link lengths (mm)
  float ars=59.13,brs=-3.5,crs=236; //position of CS1 w.r.t CS fixed to the body's base

// Geometric parameters for left shoulder
  float lls1=43.87, lls2=178.8, lls3=221.2; //link lengths (mm)
  float als=-59.13,bls=-3.5,cls=236; //position of CS1 w.r.t CS fixed to the body's base
  
//end point with respect to CS0
  Point P0;

// Constants and geometric paramers
  float pi=3.14;
  float COUNT_TO_RAD=(pi/2)/2048; //Convert encoder counts to radians (radians/count)
  Transformation H23, H12, H01;
  Point P3;

void setup() {
  // Initiate serial communication
    Serial.begin(9600);
    SPI.begin();
  // Define CS origins and the POI wrt the adjacent CS    
    H23.p=leftarm.CS_ORIGINS(lrs2, 0, 0);
    H12.p=leftarm.CS_ORIGINS(lrs1, 0,0);
    H01.p=leftarm.CS_ORIGINS(ars, brs, crs);
    P3=leftarm.CS_ORIGINS(lrs3,0,0);
}

void loop() {
  delay(100);

  // Relative angle of joints (rad)
    ENC8_ANGLE_RAD=-enc8.read()*COUNT_TO_RAD;
    ENC9_ANGLE_RAD=-enc9.read()*COUNT_TO_RAD;
    ENC10_ANGLE_RAD=-enc10.read()*COUNT_TO_RAD;

  // Print the relative angles
    leftarm.PRINT_ANGLE(ENC8_ANGLE_RAD, ENC9_ANGLE_RAD, ENC10_ANGLE_RAD);

  // HTM matrix for relative joint angles - References page 132 of notebook
    H23.R=leftarm.ROTATION_MATRIX(0,0,ENC10_ANGLE_RAD);
    H12.R=leftarm.ROTATION_MATRIX(0,0,ENC9_ANGLE_RAD);
    H01.R=leftarm.ROTATION_MATRIX(ENC8_ANGLE_RAD,0,0);


  // End point w.r.t. CS0
    P0=leftarm.FORWARD_KINEMATICS(H01, H12, H23, P3);
    Serial <<"End point of limb relative to CS0 = "<< P0 <<" (mm)" << '\n';
}

Point Z_HAT_GROUND(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point GROUND_NORMAL){
  //Unit vector normal to the ground plane with respect to the body fixed coordinate system
    // Normalize this vector
      Point Z_HAT_GROUND=GROUND_NORMAL/GROUND_NORMAL.Magnitude(); //Normal to ground plane
      return Z_HAT_GROUND;
}

Point GROUND_NORMAL(Point A_MEAS,Point B_MEAS, Point C_MEAS){
  //Unit vector normal to the ground plane with respect to the body fixed coordinate system
    // Normal vector to the plane of the ground is the corss of vector AB and AC
      Point AB=A_MEAS-B_MEAS;
      Point AC=A_MEAS-C_MEAS;
      Point GROUND_NORMAL=AC.CrossProduct(AB);
      return GROUND_NORMAL;
}

Point D(Point A_MEAS, Point GROUND_NORMAL, Point D_MEAS){
    //I will solve for D based on the MEASured direction of the vector D_MEAS. This is so that the point I use for D is sure to be in the previously defined plane. (Page 144 of notebook)
      float e=A_MEAS.DotProduct(GROUND_NORMAL)/D_MEAS.DotProduct(GROUND_NORMAL); //What to scale vector D_MEAS by so it intersects with the previous plane
      Point D=D_MEAS*e;
}
      
Transformation H_A_0(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point D_MEAS, Point GROUND_NORMAL,Point Z_HAT_GROUND){
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

Point CENTER_OF_MASS(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point D_MEAS,Point Z_HAT_GROUND,Transformation H_A_O){
  // Calculate the position of the center of mass with respect to a coordinate system fixed at P0
    // Project the origin of O onto the plane. This projection is the projected position of the CM (E)
        Point A_FRAME_A=H_A_O.p; //Express the vector [0,0,0] (in frame O) in the frame A
        Point E=A_FRAME_A-Z_HAT_GROUND*A_FRAME_A.DotProduct(Z_HAT_GROUND); //CM projected onto the ground plane
}

BLA::Matrix<3,4> CONCATENTATED_GROUND_CONTACT_POINTS(Point B_MEAS, Point C_MEAS, Point D, Transformation H_A_O){
  // Gives ground contact point wrt the CS attached to the ground. (Project all contact points onto the ground plane) THe matrix is the vectors [A,B,C,D]
    BLA::Matrix<3,1> A= {0,0,0}; //vector A wrt the CS attached to the ground
    BLA::Matrix<3,1> B= H_A_O.R*B_MEAS+H_A_O.p; //vector B wrt the CS attached to the ground
    BLA::Matrix<3,1> C= H_A_O.R*C_MEAS+H_A_O.p; //vector C wrt the CS attached to the ground
    BLA::Matrix<3,1> D_TEMP= H_A_O.R*D+H_A_O.p; //vector D wrt the CS attached to the ground
    BLA::Matrix<3,4> GCP=A||B||C||D_TEMP;
}

