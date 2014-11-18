// GenericRoverManeuvre.c

// include the constants
#include "constants.h"

// include the header
#include "GenericRoverManoeuvre.h"

#include <stdio.h>
#include <math.h>


/* ------------------------ */
/* -- Generic Ackermann  -- */
/* ------------------------ */

EXPORT int GenericAckermann( ROVER_PARAM *MyRover,
	double RoverVelocity,
	double *RotationCenter,
	double *PointToControl,
	double *WheelSteering,
	double *WheelVelocity )
{
	/* ---  declare variables  --- */
	int i=0;		// 'for' loops variable
	// coordinates of the rotation center in the rover frame
	//double Rx = RotationCenter[X];
	//double Ry = RotationCenter[Y];
	// Rover angular velocity in [rad/s] corresponding to the linear velocity
	double RoverAngularVelocity = 0.;
	// distance between RotationCenter and PoinToControl
	double DistRC_PTC = 0.;
	//static int PreviousState;		// variable to 'remember' if the 

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->GenericAckermann()\n" );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->GenericAckermann() : MyRover is NULL\n\n" );
		return -1;
	} else if( RotationCenter == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->GenericAckermann() : RotationCenter is NULL\n\n" );
		return -1;
	}else if( PointToControl == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->GenericAckermann() : RoverPointToControl is NULL\n\n" );
		return -1;
	} else if( WheelVelocity == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->GenericAckermann() : WheelVelocity is NULL\n\n" );
		return -1;
	} else if( WheelSteering == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->GenericAckermann() : WheelSteering is NULL\n\n" );
		return -1;
	}

	/* ---  set wheel angle  --- */
	// the point to control influence only the wheel speed, wheel angle only set by rotation center
	// different cases if Ry > or < 0 in order to turn RIGHT with positive linear velocity
	// note : if the rotation center is on the X axis of the rover, it will turn LEFT

	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the steering wheel
		if( MyRover->IsSteeringWheel[i] == TRUE )
		{
			double x = MyRover->WheelCoordCart[i][X];
			double y = MyRover->WheelCoordCart[i][Y];

			double Rx = RotationCenter[X];
			double Ry = RotationCenter[Y];

			// handle the case of Ry<0
			// only flip the Y axis and inverse the sign of the angle to have the right angles
			y *= SGN(RotationCenter[Y]);
			Ry *= SGN(RotationCenter[Y]);

			WheelSteering[i] = atan2( x-Rx, Ry-y );	// atan2(opp, hyp)
			// handle the case of Ry<0
			WheelSteering[i] *= SGN(RotationCenter[Y]);
			//WheelSteering[i] = SGN(Ry) * atan2( x-Rx, ABS(Ry-y) ) * RAD2DEG;	// atan2(opp, hyp) ;  CORRECT ONLY IF RotationCenter is OUTSIDE THE ROVER
			//printf( "\tx-Rx = %f\n", x-Rx );
			//printf( "\tRy-y = %f\n", Ry-y );
			//printf( "\n" );
			//WheelSteering[i] = 0.;
		}
		else
		{
			//printf( "%d NoSpeed\n", i );
			WheelSteering[i] = 0.;
		} 
	}


	/* ---  set wheel angular velocity  --- */
	// note : the input RoverLinearVelocity is the linear velocity of the point to control ;
	// i.e. if the point to control is (0;0), the RoverLinearVelocity is the linear velocity of the rover center


	// compute rover angular velocity
	DistRC_PTC = Dist2( RotationCenter[X], RotationCenter[Y], PointToControl[X], PointToControl[Y] );
	// check case of point turn
	if( DistRC_PTC == 0. )
		RoverAngularVelocity = RoverVelocity;
	else
	{
		// only if RotationCenter != RoverPointToControl, i.e. DistRC_PTC != 0.
		RoverAngularVelocity = RoverVelocity / DistRC_PTC;
	}

#ifdef DEBUG
	printf( "RoverAngularVelocity = %3.2f rad/s\n", RoverAngularVelocity );
	printf( "Wheel Velocity (Linear | Angular) :\n" );
#endif

	// compute wheel velocity
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the driving wheel
		//printf( "drive : %d\n", MyRover->IsDrivingWheel[i] );
		if( MyRover->IsDrivingWheel[i] == TRUE )
		{
			double x = MyRover->WheelCoordCart[i][X];
			double y = MyRover->WheelCoordCart[i][Y];
			double Rx = RotationCenter[X];
			double Ry = RotationCenter[Y];

			double WheelLinVelocity = RoverAngularVelocity * Dist2( x, y, Rx, Ry );
			WheelVelocity[i] = WheelLinVelocity / MyRover->WheelRadius[i];
#ifdef DEBUG
			printf( "\t%3.2f\t|\t%3.2f\n", WheelLinVelocity, WheelVelocity[i] );
#endif
		}
		else
		{
			//printf( "%d NoSpeed\n", i );
			WheelVelocity[i] = 0.;
#ifdef DEBUG
			printf( "\t%3.2f\t|\t%3.2f\n", 0., 0. );
#endif
		} 
	}

	return 0;
}


/* ------------------------ */
/* -- Crab               -- */
/* ------------------------ */

EXPORT int Crab( ROVER_PARAM *MyRover,
	double RoverLinearVelocity,
	double HeadingAngle,
	double *WheelSteering,
	double *WheelVelocity )
{
	/* ---  declare variables  --- */
	int i=0;		// 'for' loops variable

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->Crab()\n" );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->Crab() : MyRover is NULL\n\n" );
		return -1;
	}
	// check the ouput pointers
	else if( WheelSteering == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->Crab() : WheelSteering is NULL\n\n" );
		return -1;
	}
	else if( WheelVelocity == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->Crab() : WheelVelocity is NULL\n\n" );
		return -1;
	}

	// set the wheel steering
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the steering wheels
		if( MyRover->IsSteeringWheel[i] == TRUE )
			WheelSteering[i] = HeadingAngle;
		else
		{
			WheelSteering[i] = 0;
		}
	}

	// set the wheel speed
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the driving wheels
		if( MyRover->IsDrivingWheel[i] == TRUE )
			WheelVelocity[i] = RoverLinearVelocity / (MyRover->WheelRadius[i]);
		else
		{
			WheelVelocity[i] = 0.;
		}
	}

	return 0;
}


/* ------------------------ */
/* -- SpotTurn           -- */
/* ------------------------ */

EXPORT int SpotTurn( ROVER_PARAM *MyRover,
	double RoverAngularVelocity,
	double *WheelSteering,
	double *WheelVelocity )
{
	/* ---  declare variables  --- */
	int i=0;		// 'for' loops variable
	int DirFactor = -1;		// factor to give the direction of the velocity for each wheel (not all wheels turn in the same direction)
	// (-1 -> turn bwd ; +1 -> turn fwd for RoverAngularVelocity > 0, i.e. rover turns LEFT)

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->SpotTurn()\n" );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->SpotTurn() : MyRover is NULL\n\n" );
		return -1;
	}
	else if( WheelVelocity == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->SpotTurn() : WheelVelocity is NULL\n\n" );
		return -1;
	}
	else if( WheelSteering == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->SpotTurn() : WheelSteering is NULL\n\n" );
		return -1;
	}


	/* ---  set wheel angle  --- */
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the steering wheel
		if( MyRover->IsSteeringWheel[i] == TRUE )
		{
			//WheelSteering[i] = MyRover->WheelCoordPol[i][ALPHA] - SGN(MyRover->WheelCoordPol[i][ALPHA])*M_PI;
			//WheelSteering[i] = MyRover->WheelCoordPol[i][ALPHA] + M_PI/2.;

			double x = MyRover->WheelCoordCart[i][X];
			double y = MyRover->WheelCoordCart[i][Y];
			WheelSteering[i] = atan2( x, -y );	// atan2(opp, hyp)
		}
		else
		{
			//printf( "%d NoSpeed\n", i );
			WheelSteering[i] = 0;
		} 
	}

	/* ---  set wheel velocity  --- */
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the driving wheel
		//printf( "drive : %d\n", MyRover->IsDrivingWheel[i] );
		if( MyRover->IsDrivingWheel[i] == TRUE )
			//WheelVelocity[i] = DirFactor*RoverAngularVelocity*MyRover->WheelCoordPol[i][D]/MyRover->WheelRadius[i];
			WheelVelocity[i] = RoverAngularVelocity*MyRover->WheelCoordPol[i][D]/MyRover->WheelRadius[i];	
		else
		{
			//printf( "%d NoSpeed\n", i );
			WheelVelocity[i] = 0.;
		} 
		// update the velocity direction factor
		DirFactor *= -1;
	}

	return 0;
}




/* ------------------------------- */
/* -- Distance Between 2 Points -- */
/* ------------------------------- */

double Dist2( double x1, double y1,
	double x2, double y2 )
{
	double dx = x2 - x1;
	double dy = y2 - y1;

	double dist = sqrt( dx*dx + dy*dy );

	return dist;
}


/* ------------------------ */
/* -- ConvCoordCart2Pol  -- */
/* ------------------------ */

EXPORT void ConvCoordCart2Pol( const double CoordCart[][2],	
	double CoordPol[][2],
	int NumCoord
	)
{
	int k=0;
	int i=0;
	
#ifdef DEBUG
	printf( "in GenericRoverManoeuvre.c->ConvCoordPol2Cart():\n" );
#endif

	//MyRover->WheelCoordCart[0][X] = 123.;
	//MyRover->WheelCoordCart[1][X] = 456.;

	//// set the polar coordinates
	//for( i=0 ; i<NumCoord ; i++ )
	//{
	//	CoordPol[i] = CoordCart[k++];
	//	CoordPol[i+1] = CoordCart[k++];
	//}

	// compute the cartesian coordinates
	// the angle and the distance are computed from the cartesian coordinates
	for( i=0 ; i<NumCoord ; i++ )
	{
		//MyRover->WheelRoverCenterAngle[i] = MyRoverParameters[k++];
		//MyRover->WheelRoverCenterAngle[i] = atan2( MyRover->WheelCoord[i][Y], MyRover->WheelCoord[i][X] );
		// Rho polar coordinate
		CoordPol[i][0] =   sqrt( CoordCart[i][0]*CoordCart[i][0] + CoordCart[i][1]*CoordCart[i][1] );
		// Theta polar coordinate
		CoordPol[i][1] = atan2( CoordCart[i][1], CoordCart[i][0] );
	}
	
}


/* ------------------------ */
/* -- ConvCoordPol2Cart  -- */
/* ------------------------ */

EXPORT void ConvCoordPol2Cart( const double CoordPol[][2],	
	double CoordCart[][2],
	int NumCoord
	)
{
	int k=0;
	int i=0;
	
#ifdef DEBUG
	printf( "in GenericRoverManoeuvre.c->ConvCoordPol2Cart():\n" );
#endif

	//MyRover->WheelCoordCart[0][X] = 123.;
	//MyRover->WheelCoordCart[1][X] = 456.;

	//// set the polar coordinates
	//for( i=0 ; i<NumCoord ; i++ )
	//{
	//	CoordPol[i] = CoordCart[k++];
	//	CoordPol[i+1] = CoordCart[k++];
	//}

	// compute the cartesian coordinates
	// the angle and the distance are computed from the cartesian coordinates
	for( i=0 ; i<NumCoord ; i++ )
	{
		//MyRover->WheelRoverCenterAngle[i] = MyRoverParameters[k++];
		//MyRover->WheelRoverCenterAngle[i] = atan2( MyRover->WheelCoord[i][Y], MyRover->WheelCoord[i][X] );
		// X cartesian coordinate
		CoordCart[i][0] =   CoordPol[i][0] * cos( CoordPol[i][1] );
		// Y cartesian coordinate
		CoordCart[i][1] = CoordPol[i][0] * sin( CoordPol[i][1] );
	}
}


/* ------------------------ */
/* -- RoverInit          -- */
/* ------------------------ */

EXPORT int RoverInit( ROVER_PARAM *MyRover, 
	const double *MyRoverParameters, 
	int NumParamIn )
{
	/* ---  declare variables  --- */
	int i=0;		// 'for' loops variable
	int k=0;		// variable for the rover parameters input
	int WheelCoordType;	// for type of wheel coordinate (CARTESIAN, POLAR or BOTH)
	int WalkCoordType;	// for type of walk joint coordinate (CARTESIAN, POLAR or BOTH)

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuver.c->RoverInit()\n" );
	printf( "number of input parameters : %d\n", NumParamIn );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->RoverInit() : MyRover is NULL\n\n" );
		return -1;
	}
	else if( MyRoverParameters == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->RoverInit() : MyRoverParameters is NULL\n\n" );
		return -1;
	}

	/* --- WheelNumber  --- */
	MyRover->WheelNumber = (int) MyRoverParameters[k++];

	//	/* --- WalkingWheelNumber  --- */
	//	MyRover->WalkingWheelNumber = (int) MyRoverParameters[k++];	
	//
	//	/* --- SteeringWheelNumber  --- */			
	//	MyRover->SteeringWheelNumber = (int) MyRoverParameters[k++];
	//
	//	/* --- DrivingWheelNumber  --- */				
	//	MyRover->DrivingWheelNumber = (int) MyRoverParameters[k++];


	/* --- IsWalkingWheel  --- */
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		MyRover->IsWalkingWheel[i] = (int) MyRoverParameters[k++];
	}

	/* --- IsSteeringWheel  --- */
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		MyRover->IsSteeringWheel[i] = (int) MyRoverParameters[k++];
	}

	/* --- IsDrivingWheel  --- */
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		MyRover->IsDrivingWheel[i] = (int) MyRoverParameters[k++];
	}


	//	/* --- RoverWidth  --- */
	//	MyRover->RoverWidth = MyRoverParameters[k++];		


	/* --- WheelRadius  --- */
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		MyRover->WheelRadius[i] = MyRoverParameters[k++];
	}

	/* --- WheelCoord  --- */
	// check which coordinate are given (cartesian, polar, both)
	WheelCoordType = (int) (MyRoverParameters[k++]);		// retrieve walk joint coordinate type
	switch( WheelCoordType )
	{
	case CARTESIAN:
		//printf( "cartesian coordinates -> conversion CART-POL!\n\n" );
		//ConvWheelCoordCart2Pol( MyRover, (MyRoverParameters+k) );
		
		// set the wheel cartesian coordinates
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			MyRover->WheelCoordCart[i][X] = MyRoverParameters[k++];
			MyRover->WheelCoordCart[i][Y] = MyRoverParameters[k++];
		}

		// compute the wheel polar  coordinates from cartesian
		ConvCoordCart2Pol( MyRover->WheelCoordCart, MyRover->WheelCoordPol, MyRover->WheelNumber );

		// update index
		//k += 12;
		break;
	case POLAR:
		//printf( "polar coordinates -> conversion POL-CART!\n\n" );
		//ConvWheelCoordPol2Cart( MyRover, (MyRoverParameters+k) );

		// set the wheel polar coordinates
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			MyRover->WheelCoordPol[i][D] = MyRoverParameters[k++];
			MyRover->WheelCoordPol[i][ALPHA] = MyRoverParameters[k++];
		}

		// compute the wheel cartesian  coordinates from polar
		ConvCoordCart2Pol( MyRover->WheelCoordPol, MyRover->WheelCoordCart, MyRover->WheelNumber );

		// update index
		//k += 12;
		break;
	case BOTH:
		//printf( "both coordinates -> no conversion!\n\n" );

		// set the cartesian coordinates
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			MyRover->WheelCoordCart[i][X] = MyRoverParameters[k++];
			MyRover->WheelCoordCart[i][Y] = MyRoverParameters[k++];
		}

		// set the polar coordinates
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			MyRover->WheelCoordPol[i][D] = MyRoverParameters[k++];
			MyRover->WheelCoordPol[i][ALPHA] = MyRoverParameters[k++];
		}

		break;
	default:

		// set the wheel cartesian and polar coordinates to 0.
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			// cartesian
			MyRover->WheelCoordCart[i][X] = 0.;
			MyRover->WheelCoordCart[i][Y] = 0.;

			// polar
			MyRover->WheelCoordPol[i][D] = 0.;
			MyRover->WheelCoordPol[i][ALPHA] = 0.;
		}
		
		printf( "ERROR in GenericRoverManoeuvre.c->RoverInit():\n" );
		printf( "type of wheel coordinate not known!\n" );
		printf( "type %d instead of\n", WheelCoordType );
		printf( "\t%d -> CARTESIAN\n", CARTESIAN );
		printf( "\t%d -> POLAR\n", POLAR );
		printf( "\t%d -> BOTH\n\n", BOTH );

		return -1;
	}

	/*--- part related to wheel walking  ---*/

#ifdef DEBUG
	printf( "\nin GenericRoverManeouvre.c->RoverInit():\n" );
	printf( "input param pointer (k) before wheel walking : %d\n", k );
#endif

	// check number of parameters
	if( NumParamIn > 50 )
	{

		/* --- Leg Length  --- */
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			MyRover->LegLengthV[i] = MyRoverParameters[k++];
		}

		/* --- WalkCoord  --- */
		// check which coordinate are given (cartesian, polar, both)
		WalkCoordType = (int) (MyRoverParameters[k++]);		// retrieve walk joint coordinate type
		switch( WalkCoordType )
		{
		case CARTESIAN:
			//printf( "cartesian coordinates -> conversion CART-POL!\n\n" );
			//>ConvWalkCoordCart2Pol( MyRover, (MyRoverParameters+k) );

			// set the walk joint cartesian coordinates
			for( i=0 ; i<MyRover->WheelNumber ; i++ )
			{
				MyRover->WalkCoordCart[i][X] = MyRoverParameters[k++];
				MyRover->WalkCoordCart[i][Y] = MyRoverParameters[k++];
			}

			// compute the walk joint polar  coordinates from cartesian
			ConvCoordCart2Pol( MyRover->WalkCoordCart, MyRover->WalkCoordPol, MyRover->WheelNumber );
			// update index
			//k += 12;
			break;
		case POLAR:
			//printf( "polar coordinates -> conversion POL-CART!\n\n" );
			//>ConvWalkCoordPol2Cart( MyRover, (MyRoverParameters+k) );
			
			// set the walk joint polar coordinates
			for( i=0 ; i<MyRover->WheelNumber ; i++ )
			{
				MyRover->WalkCoordPol[i][D] = MyRoverParameters[k++];
				MyRover->WalkCoordPol[i][ALPHA] = MyRoverParameters[k++];
			}

			// compute the walk joint cartesian  coordinates from polar
			ConvCoordPol2Cart( MyRover->WalkCoordPol, MyRover->WalkCoordCart, MyRover->WheelNumber );
			// update index
			//k += 12;
			break;
		case BOTH:	// first cart then pol
			//printf( "both coordinates -> no conversion!\n\n" );

			// set the walk joint cartesian coordinates
			for( i=0 ; i<MyRover->WheelNumber ; i++ )
			{
				MyRover->WalkCoordCart[i][X] = MyRoverParameters[k++];
				MyRover->WalkCoordCart[i][Y] = MyRoverParameters[k++];
			}

			// set the walk joint polar coordinates
			for( i=0 ; i<MyRover->WheelNumber ; i++ )
			{
				MyRover->WalkCoordPol[i][D] = MyRoverParameters[k++];
				MyRover->WalkCoordPol[i][ALPHA] = MyRoverParameters[k++];
			}

			break;
		default:

			// set the walk joint cartesian and polar coordinates to 0.
			for( i=0 ; i<MyRover->WheelNumber ; i++ )
			{
				// cartesian
				MyRover->WalkCoordCart[i][X] = 0.;
				MyRover->WalkCoordCart[i][Y] = 0.;

				// polar
				MyRover->WalkCoordPol[i][D] = 0.;
				MyRover->WalkCoordPol[i][ALPHA] = 0.;
			}

			printf( "ERROR in GenericRoverManoeuvre.c->RoverInit():\n" );
			printf( "type of walk joint coordinate not known!\n" );
			printf( "type %d instead of\n", WalkCoordType );
			printf( "\t%d -> CARTESIAN\n", CARTESIAN );
			printf( "\t%d -> POLAR\n", POLAR );
			printf( "\t%d -> BOTH\n\n", BOTH );

			return -1;
		}
	}
	else
	{
		// no walk parameters, set to 0
		/* --- Leg Length  --- */
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			// leg length
			MyRover->LegLengthV[i] = 0.;
			// walk cart coord
			MyRover->WalkCoordCart[i][X] = 0.;
			MyRover->WalkCoordCart[i][Y] = 0.;
			// walk pol coord
			MyRover->WalkCoordPol[i][D] = 0.;
			MyRover->WalkCoordPol[i][ALPHA] = 0.;
		}
	}


	return k;		// start from 0 so it is ok (number of parameters to initialize)
}


/* ------------------------ */
/* -- RoverPrint         -- */
/* ------------------------ */

EXPORT int RoverPrint( ROVER_PARAM *MyRover )
{
	/* ---  declare variables  --- */
	int i=0;		// 'for' loops variable

#ifdef DEBUG
	printf( "\nin GenericRoverManeuver.c->RoverPrint()\n" );

#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
#ifdef DEBUG
		printf( "\tERROR in GenericRoverManeuver.c->RoverPrint() : MyRoverParameters is NULL\n\n" );
#endif
		return -1;
	}

	/* --- Wheel Number  --- */
	printf( "\tWheelNumber [-]:\n\t\t%d\n", MyRover->WheelNumber );
	//	printf( "\tWalkingWheelNumber :\n\t\t%d\n", MyRover->WalkingWheelNumber );
	//	printf( "\tSteeringWheelNumber :\n\t\t%d\n", MyRover->SteeringWheelNumber );
	//	printf( "\tDrivingWheelNumber :\n\t\t%d\n", MyRover->DrivingWheelNumber );

	/* --- Walking Wheel  --- */
	printf( "\tIsWalkingWheel (1->true, 0->false) :\n\t" );
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t%d", MyRover->IsWalkingWheel[i] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );

	/* --- Steering Wheel  --- */
	printf( "\tIsSteeringWheel (1->true, 0->false) :\n\t" );
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t%d", MyRover->IsSteeringWheel[i] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );

	/* --- Driving Wheel  --- */
	printf( "\tIsDrivingWheel (1->true, 0->false) :\n\t" );
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t%d", MyRover->IsDrivingWheel[i] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );


	/* --- Wheel Radius  --- */
	//	printf( "\tRoverWidth :\n\t\t%f\n", MyRover->RoverWidth );
	printf( "\tWheelRadius [m] :\n\t" );
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t%f", MyRover->WheelRadius[i] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );

	/* --- WheelCoord Cart  --- */
	printf( "\tWheelCoordCart (x;y) [m] [m] :\n\t" ); 
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t(%f;%f)", MyRover->WheelCoordCart[i][X], MyRover->WheelCoordCart[i][Y] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );

	/* --- WheelCoord Pol  --- */
	printf( "\tWheelCoordPol (rho;theta) [m] [rad] :\n\t" );
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t(%f;%f)", MyRover->WheelCoordPol[i][D], MyRover->WheelCoordPol[i][ALPHA] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );


	/* --- Leg length  --- */
	printf( "\tLeg Length [m] :\n\t" );
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t%f", MyRover->LegLengthV[i] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );

	/* --- WalkCoord Cart  --- */
	printf( "\tWalkCoordCart (x;y) [m] [m] :\n\t" ); 
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t(%f;%f)", MyRover->WalkCoordCart[i][X], MyRover->WalkCoordCart[i][Y] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );


	/* --- WalkCoord Pol  --- */
	printf( "\tWalkCoordPol (rho;theta) [m] [rad] :\n\t" );
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		printf( "\t(%f;%f)", MyRover->WalkCoordPol[i][D], MyRover->WalkCoordPol[i][ALPHA] );
		if( i%2 == 1 )
			printf( "\n\t" );
	}
	printf( " \n" );

	//printf( "\tWheelRoverCenterAngle :\n" );
	//for( i=0 ; i<MyRover->WheelNumber ; i++ )
	//{
	//	printf( "\t\t%f [deg]\n", MyRover->WheelRoverCenterAngle[i]*RAD2DEG );
	//}
	//printf( "\tWheelRoverCenterDist :\n" ); 
	//for( i=0 ; i<MyRover->WheelNumber ; i++ )
	//{
	//	printf( "\t\t%f [m]\n", MyRover->WheelRoverCenterDist[i] );
	//}

	//printf( "\tWheelWheelDist :\n" );
	//for( i=0 ; i<MyRover->WheelNumber/2 ; i++ )
	//{
	//	printf( "\t\t%f\n", MyRover->WheelWheelDist[i] );
	//}



	//	// rover limitations
	//	printf( "\n" );
	//	printf( "\tWalkingAngleMax :\n" );
	//		for( i=0 ; i<MyRover->WheelNumber ; i++ )
	//		{
	//			printf( "\t\t%f [deg]\n", MyRover->WalkingAngleMax[i]*RAD2DEG );
	//		}
	//	printf( "\tWalkingAngleMin :\n" ); 
	//		for( i=0 ; i<MyRover->WheelNumber ; i++ )
	//		{
	//			printf( "\t\t%f [deg]\n", MyRover->WalkingAngleMin[i]*RAD2DEG );
	//		}
	//	printf( "\tSteeringAngleMax :\n" ); 
	//		for( i=0 ; i<MyRover->WheelNumber ; i++ )
	//		{
	//			printf( "\t\t%f [deg]\n", MyRover->SteeringAngleMax[i]*RAD2DEG );
	//		}
	//	printf( "\tSteeringAngleMin :\n" );
	//		for( i=0 ; i<MyRover->WheelNumber ; i++ )
	//		{
	//			printf( "\t\t%f [deg]\n", MyRover->SteeringAngleMin[i]*RAD2DEG );
	//		}



	return 0;
}


/* ------------------------ */
/* -- RoverReset         -- */
/* ------------------------ */

EXPORT int RoverReset( ROVER_PARAM *MyRover )
{
	/* ---  declare variables  --- */
	int i=0;		// 'for' loops variable

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->RoverReset()\n" );
#endif


	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
#ifdef DEBUG
		printf( "\tERROR in GenericRoverManeuver.c->RoverReset() : MyRoverParameters is NULL\n\n" );
#endif
		return -1;
	}


	MyRover->WheelNumber = 0;				
	//	MyRover->WalkingWheelNumber = 0;			
	//	MyRover->SteeringWheelNumber = 0;		
	//	MyRover->DrivingWheelNumber = 0;		

	//	MyRover->RoverWidth = 0;		

	for( i=0 ; i<NUM_WHEEL_ROVER_MAX ; i++ )
	{

		MyRover->IsSteeringWheel[i] = 0;
		MyRover->IsWalkingWheel[i] = 0;
		MyRover->IsDrivingWheel[i] = 0;

		MyRover->WheelRadius[i] = 0.;	

		MyRover->WheelCoordCart[i][X] = 0.;
		MyRover->WheelCoordCart[i][Y] = 0.;

		MyRover->WheelCoordPol[i][D] = 0.;
		MyRover->WheelCoordPol[i][ALPHA] = 0.;
		//		MyRover->WheelRoverCenterAngle[i] = 0.;
		//		MyRover->WheelRoverCenterDist[i] = 0.;



		//		MyRover->WalkingAngleMax[i] = 0.;
		//		MyRover->WalkingAngleMin[i] = 0.;

		//		MyRover->SteeringAngleMax[i] = 0.;
		//		MyRover->SteeringAngleMin[i] = 0.;	
	}

	//MyRover->SteeringAngleMax = 0;			
	//MyRover->SteeringAngleMin = 0;			
	//MyRover->WalkingAngleMax = 0;			
	//MyRover->WalkingAngleMin = 0;		

	return 0;
}


/* ------------------------ */
/* -- SkidTurn           -- */
/* ------------------------ */

EXPORT int SkidTurn( 
	ROVER_PARAM *MyRover, 
	double RoverVelocity,
	double RadiusOfCurvature,
	int IsStraightLine,
	double *WheelVelocity )
{
	/* ---  declare variables  --- */

	int i=0;		// 'for' loops variable
	int DirFactor=-1;		// factor to give the direction of the velocity (-1 -> bwd ; +1 -> fwd for velocity >0)
	double WheelVelocityRatio = 0.;	// the left/right   velocity ratio
	//		WheelVelocityRatio =  1	=> Curvature = 0								=> straight motion
	//		WheelVelocityRatio = -1	=> Curvature = +/-inf							=> spot turn anticlockwise if velocity>0
	//		WheelVelocityRatio <  1	=> Curvature in ]-inf;0[ U ]2/RoverWidth;+inf[	=> turn right
	//		WheelVelocityRatio >  1	=> Curvature in ]0;2/RoverWidth[				=> turn left

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->SkidTurn()\n" );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->SkidTurn() : MyRover is NULL\n\n" );
		return -1;
	}
	else if( WheelVelocity == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->SkidTurn() : WheelVelocity is NULL\n\n" );
		return -1;
	}

	/* ---  Skid Straight Line  --- */
	if( IsStraightLine == TRUE )		// Spot turn ; in this case, the RoverLinearVelocity is the rover ANGULAR velocity
	{
#ifdef DEBUG
		printf( "\nin GenericRoverManoeuvre.c : SkidStraightLine\n" );
#endif

		// compute wheel speed
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			// check if wheel drive
			if( MyRover->IsDrivingWheel[i] == TRUE )
			{
				// RoverVelocity is the rover LINEAR velocity
				WheelVelocity[i] = RoverVelocity/MyRover->WheelRadius[i];
			}
			else	// wheel don't drive => wheel speed is null
			{
				WheelVelocity[i] = 0.;
			}
		}

	}
	/* ---  Skid Spot Turn  --- */
	// (RadiusOfCurvature = 0)
	else if( ABS(RadiusOfCurvature) < EPSILON )
	{
#ifdef DEBUG
		printf( "\nin GenericRoverManoeuvre.c : Skid Spot Turn\n" );
#endif
		for( i=0 ; i<MyRover->WheelNumber ; i++ )
		{
			// special case: the input rover linear velocity is meant as the rover angular velocity
			//double RoverAngularVelocity = RoverLinearVelocity;
			// check if wheel drive
			if( MyRover->IsDrivingWheel[i] == TRUE )
			{
				// distance along the Y axis between the wheel and the rover center
				// rover turn left if angular velocity > 0 => angular speed left wheel < 0
				//double ProjDistWheelRC = - MyRover->WheelCoordCart[i][Y];
				// compute wheel angular velocity
				// RoverVelocity is the rover ANGULAR velocity
				WheelVelocity[i] = RoverVelocity * (- MyRover->WheelCoordCart[i][Y]) /MyRover->WheelRadius[i];
			}
			else	// wheel don't drive => wheel speed is null
			{
				WheelVelocity[i] = 0.;
			}
		}
	}
	/* ---  Skid Turn  --- */
	else
	{
#ifdef DEBUG
		printf( "\nin GenericRoverManoeuvre.c : Skid Turn\n" );
#endif

		// assumption : compute wheel speed as if all the wheels where on the axis rover center - center of rotation (only Y component)

		// compute wheels velocity
		for( i=0 ; i<MyRover->WheelNumber ; i++ )		
		{
			// compute the rover angular velocity
			// RoverVelocity is the rover LINEAR velocity
			double RoverAngularVelocity = RoverVelocity / RadiusOfCurvature;
			// check if wheel drive
			if( MyRover->IsDrivingWheel[i] == TRUE )
			{
				// projection on Y axis of the distance wheel center - rover center
				// always '-' because sign(alpha) changes between left and right wheel!
				double ProjDistWheelRoC = RadiusOfCurvature - MyRover->WheelCoordCart[i][Y];	
				// compute wheel angular velocity
				WheelVelocity[i] = RoverAngularVelocity * ProjDistWheelRoC / MyRover->WheelRadius[i];
			}
			else	// wheel don't drive => wheel speed is null
			{
				WheelVelocity[i] = 0.;
			}
		}
	}

	return 0;
}


/* ------------------------ */
/* -- Stop               -- */
/* ------------------------ */

EXPORT int Stop( ROVER_PARAM *MyRover,
	double *WheelVelocity )
{
	/* ---  declare variables  --- */
	int i=0;		// 'for' loops variable

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->Stop()\n" );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->Stop() : MyRover is NULL\n\n" );
		return -1;
	}
	else if( WheelVelocity == NULL )
	{
		printf( "\tERROR in GenericRoverManeuver.c->Stop() : WheelVelocity is NULL\n\n" );
		return -1;
	}
	//else if( WheelSteering == NULL )
	//{
	//	printf( "\tERROR in GenericRoverManeuver.c > SpotTurn : WheelSteering is NULL\n\n" );
	//	return -1;
	//}

	// set velocity
	for( i=0 ; i<MyRover->WheelNumber ; i++ )		
	{
		WheelVelocity[i] = 0.;
		//SteeringAngles[i] = 0.;
	}

	return 0;
}


/* ------------------------ */
/* -- WheelWalk          -- */
/* ------------------------ */

//EXPORT int WheelWalk( ROVER_PARAM *MyRover,
//	double Distance,		
//	double LinearVelocity,	
//	double StepLength,		
//	int Gait,				
//	double *WalkVelocity,	
//	double *WheelVelocity	
EXPORT int WheelWalk( ROVER_PARAM *MyRover,	
	double *StepLength,
	int Gait,				
	double *WalkAngleRad,	
	double *WheelAngleRad	
	)
{

	// for loop variable
	int i=0;

	// variables for walking
	double A;
	double J;
	double AR;
	double SF;
	double SC;
	double SR;
	double B;
	double VC;
	double DC;
	double dC;
	double deltaC1;
	double deltaC2;
	double beta;
	double betap;
	double theta;
	double alphap;
	double alpha;
	double VR;
	double DR;
	double dR;
	double deltaR1;
	double deltaR2;
	double gammap;
	double gamma;
	double phi;

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->WheelWalk\n" );
	printf( "inputs:\n" );
//	printf( "\tdistance [m]: %f\n", Distance );
//	printf( "\tlinear velocity [m/s]: %f\n", LinearVelocity );
	printf( "\tstep length [m]: %f\n", StepLength );
	printf( "\tgait id : %d\n", Gait );
	printf( "WARNING: function not implemented!\n\n" );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\nERROR in GenericRoverManeuver.c->WheelWalk() :\nMyRover is NULL\n\n" );
		return -1;
	} else if( StepLength==NULL )
	{
		printf( "\nERROR in GenericRoverManeuver.c->WheelWalk() :\nStepLength is NULL\n\n" );
		return -1;
//	} else if( WalkVelocity == NULL )
	} else if( WalkAngleRad == NULL )
	{
		printf( "\nERROR in GenericRoverManeuver.c->WheelWalk() :\nWalkVelocity is NULL\n\n" );
		return -1;
//	}else if( WheelVelocity == NULL )
	}else if( WheelAngleRad == NULL )
	{
		printf( "\nERROR in GenericRoverManeuver.c->WheelWalk() :\nWheelVelocity is NULL\n\n" );
		return -1;
	}

	A = ABS( MyRover->WalkCoordCart[W1][X] - MyRover->WalkCoordCart[W3][X] );
	J = A/2.;
	AR = ABS( MyRover->WalkCoordCart[W3][X] - MyRover->WalkCoordCart[W5][X] ) + J;
	SF = StepLength[0];
	SC = StepLength[1];
	SR = StepLength[2];
	B = MyRover->LegLengthV[0];
	

	// compute auxiliary variables
	VC = sqrt( B*B - SF*SF );
	DC = A - SC;
	dC = sqrt( VC*VC + DC*DC );

	deltaC1 = atan( VC/DC );
	deltaC2 = acos( (B*B + dC*dC - A*A)/(2*B*dC) );

	//
	beta = acos( (A*A+B*B-dC*dC)/(2*A*B) );
	//
	betap = M_PI/2. - deltaC1 - deltaC2;

	theta = M_PI/2. + betap - beta;

	//
	alphap = asin( -SF/B );
	//
	alpha = M_PI/2. + alphap - theta;


	VR = VC + J*sin(theta);
	DR = AR + J*(1-cos(theta)) - SR;
	dR = sqrt( VR*VR + DR*DR );

	deltaR1 = atan( VR/DR );
	deltaR2 = acos( (B*B+dR*dR-AR*AR)/(2*B*dR) );

	//
	gammap = M_PI/2. - deltaR1 - deltaR2;
	//
	gamma = acos( (AR*AR+B*B-dR*dR)/(2*AR*B) );

	phi = M_PI/2. + gammap - gamma;

	//>>> DEBUG
	printf( "GenericRoverManoeuvre.c->WheelWalk():\n" );
	printf( "\tA = %f\n", A );
	printf( "\tAR = %f\n", AR );
	printf( "\tB = %f\n", B );
	printf( "\tJ = %f\n", J );
	printf( "\tSF = %f\n", SF );
	printf( "\tSC = %f\n", SC );
	printf( "\tSR = %f\n", SR );
	printf( "\n" );

	printf( "\talpha = %f [rad] = %f [deg]\n", alpha, alpha*RAD2DEG );
	printf( "\talphap = %f [rad] = %f [deg]\n", alphap, alphap*RAD2DEG );
	printf( "\n" );

	printf( "\tbeta = %f [rad] = %f [deg]\n", beta, beta*RAD2DEG );
	printf( "\tbetap = %f [rad] = %f [deg]\n", betap, betap*RAD2DEG );
	printf( "\n" );

	printf( "\tgamma = %f [rad] = %f [deg]\n", gamma, gamma*RAD2DEG );
	printf( "\tgammap = %f [rad] = %f [deg]\n", gammap, gammap*RAD2DEG );
	printf( "\n\n" );

	printf( "\tVC = %f [m]\n", VC );
	printf( "\tDC = %f [m]\n", DC );
	printf( "\tdC = %f [m]\n", dC );
	printf( "\n" );

	printf( "\tdeltaC1 = %f [rad] = %f [deg]\n", deltaC1, deltaC1*RAD2DEG );
	printf( "\tdeltaC2 = %f [rad] = %f [deg] \n", deltaC2, deltaC2*RAD2DEG );
	printf( "\n" );

	printf( "\ttheta = %f [rad] = %f [deg]\n", theta, theta*RAD2DEG );
	printf( "\n" );

	printf( "\tVR = %f [m]\n", VR );
	printf( "\tDR = %f [m]\n", DR );
	printf( "\tdR = %f [m]\n", dR );
	printf( "\n" );

	printf( "\tdeltaR1 = %f [rad] = %f [deg]\n", deltaR1, deltaR1*RAD2DEG );
	printf( "\tdeltaR2 = %f [rad] = %f [deg]\n", deltaR2, deltaR2*RAD2DEG );
	printf( "\n" );

	printf( "\tphi = %f [rad] = %f [deg]\n", phi, phi*RAD2DEG );
	printf( "\n\n" );

	// loop over all legs	
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the driving wheel
		//printf( "drive : %d\n", MyRover->IsDrivingWheel[i] );
		if( MyRover->IsWalkingWheel[i]==TRUE )
		{
			// update walk angle front wheel
			if( i < 2 )
			WalkAngleRad[i] = alpha;
			// update wheel angle centre wheel
			if( (i>=2) && (i<4) )
			WalkAngleRad[i] = beta;
			// update wheel angle centre wheel
			if( (i>=4) && (i<6) )
			WalkAngleRad[i] = gamma;
		}
		if( (MyRover->IsDrivingWheel[i] == TRUE) )
		{
			// update wheel angle front wheel
			if( i < 2 )
			WheelAngleRad[i] = alphap;
			// update wheel angle centre wheel
			if( (i>=2) && (i<4) )
			WheelAngleRad[i] = betap;
			// update wheel angle centre wheel
			if( (i>=4) && (i<6) )
			WheelAngleRad[i] = gammap;
		}

	}

	return 0;
}


/* ------------------------ */
/* -- Low CoG            -- */
/* ------------------------ */

EXPORT int LowCog( ROVER_PARAM *MyRover,
	double BodyZVelocity, 
	double BodyHeightDispl, 
	double *WalkVelocity, 
	double *WheelVelocity )
{	
	
	// for loop variable
	int i=0;

#ifdef DEBUG
	printf( "\nin GenericRoverManoeuvre.c->LowCog()\n" );
	printf( "inputs:\n\tbody Z vel %3.2f [m]\n", BodyZVelocity );
	printf( "\tbody Z displ : %3.2f [m]\n", BodyHeightDispl );
#endif

	// check the input Rover pointer is valid
	if( MyRover == NULL )
	{
		printf( "\nERROR in GenericRoverManeuver.c->LowCog() :\nMyRover is NULL\n\n" );
		return -1;
	} else if( WalkVelocity == NULL )
	{
		printf( "\nERROR in GenericRoverManeuver.c->LowCog() :\nWalkVelocity is NULL\n\n" );
		return -1;
	}else if( WheelVelocity == NULL )
	{
		printf( "\nERROR in GenericRoverManeuver.c->LowCog() :\nWheelVelocity is NULL\n\n" );
		return -1;
	}


	// loop over all legs	
	for( i=0 ; i<MyRover->WheelNumber ; i++ )
	{
		// check the driving wheel
		//printf( "drive : %d\n", MyRover->IsDrivingWheel[i] );
		if( (MyRover->IsDrivingWheel[i] == TRUE) && ( MyRover->IsWalkingWheel[i]==TRUE) )
		{
			// compute joint velocity
			double WheelVel = 0.;

			// check input validity (e.g. final body length reachable)
			if( ABS(BodyHeightDispl) <= (MyRover->LegLengthV[i] + MyRover->WheelRadius[i]) )
			{
				// leg height (proj on Z axis of the distance between wheel centre and walk centre)
				double Lh = MyRover->LegLengthV[i];
				// leg width (proj on X axis of the distance between wheel centre and walk centre)
				double Lw = MyRover->WheelCoordCart[i][X] - MyRover->WalkCoordCart[i][X];
				// leg length (distance between wheel centre and walk centre)
				double Ll = sqrt( Lh*Lh + Lw*Lw );

				// body final height with respect to wheel centre
				double BFHw = Lh - ABS(BodyHeightDispl);

				// joint velocity
				WheelVel = - BodyZVelocity / sqrt( Ll*Ll - BFHw*BFHw );

#ifdef DEBUG
				printf( "\n\tLh = %3.2f [m]\n", Lh );
				printf( "\tLw = %3.2f [m]\n", Lw );
				printf( "\tLl = %3.2f [m]\n", Ll );
				printf( "\tBFHw = %3.2f [m]\n", BFHw );
#endif
			}
			else
			{
				WheelVel = 0.;
			}

			// update walk velocity and wheel velocity
			// walk -> fwd
			WalkVelocity[i] = - WheelVel;
			// wheel -> bwd
			WheelVelocity[i] = WheelVel;

#ifdef DEBUG
			printf( "\twheel vel : %3.2f [rad/s]\n", WheelVel );
			
#endif
		}
		else
		{
			// no motion (wheel nor walk)
			WalkVelocity[i] = 0.;
			WheelVelocity[i] = 0.;

#ifdef DEBUG
			printf( "\twheel vel : %3.2f [rad/s]\n", 0. );
#endif
		} 
	}


	return 0;
}