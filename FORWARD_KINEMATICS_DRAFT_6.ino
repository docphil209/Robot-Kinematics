// Here I confirm that forward kinematics works on the Arduino. This formulation of coordinate systems is less messy.


//Library for robot kinematics
  #include <ROBOT.h>

  
//Libraries for linear algebra and 3D body representations on Arduino
  #include <BasicLinearAlgebra.h>
  using namespace BLA;
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
