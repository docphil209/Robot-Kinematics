/*
	Robots.h - Library for finding the point of interest of an open kinematic chain.
	Created by Phillip H. Daniel, June 25, 2018
	Private software
*/

#ifndef ROBOT_h
#define ROBOT_h

#include <BasicLinearAlgebra.h>
#include <Geometry.h>
#include "Arduino.h"

class ROBOT
{
	public:
		// Functions
			ROBOT(int encPin8, int encPin9, int encPin10);
			Rotation ROTATION_MATRIX(float ROT_X,float ROT_Y,float ROT_Z);
			Point FORWARD_KINEMATICS(Transformation H01,Transformation H12, Transformation H23, Point P3);
			Point CS_ORIGINS(float x, float y, float z);
			void PRINT_ANGLE(float THETA_1,float THETA_2,float THETA_3);
		// Parameters and constants
	private:
};

#endif