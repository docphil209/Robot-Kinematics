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
			Point Z_HAT_GROUND(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point GROUND_NORMAL);
			Point GROUND_NORMAL(Point A_MEAS,Point B_MEAS, Point C_MEAS);
			Point D(Point A_MEAS, Point GROUND_NORMAL, Point D_MEAS);
			Transformation H_A_0(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point D_MEAS, Point GROUND_NORMAL,Point Z_HAT_GROUND);
			Point CENTER_OF_MASS(Point A_MEAS,Point B_MEAS, Point C_MEAS, Point D_MEAS,Point Z_HAT_GROUND,Transformation H_A_O);
			BLA::Matrix<3,4> CONCATENTATED_GROUND_CONTACT_POINTS(Point B_MEAS, Point C_MEAS, Point D, Transformation H_A_O);
		// Parameters and constants
	private:
};

#endif
