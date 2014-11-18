// GenericRoverManeuver.h


// define the variable of the file
#ifndef GENERIC_ROVER_MANOEUVRE_H
#define GENERIC_ROVER_MANOEUVRE_H

#ifndef NUM_WHEEL_ROVER_MAX
	#define NUM_WHEEL_ROVER_MAX 6	//!< Constant to define the maximal number of wheels (for coding purposes only).
#endif

// define the export mode (C or C++)
#ifdef WIN32	
	#ifdef __cplusplus
		#define EXPORT extern "C" __declspec(dllexport)		//!< Define the external function (WIN32 and C++) 
	#else
		#define EXPORT __declspec(dllexport)				//!< Define the external function (WIN32 and C)
	#endif
#else
	#ifdef __cplusplus
		#define EXPORT extern "C"							//!< Define the external function (LINUX and C++)
	#else
        	#define EXPORT									//!< Define the external function (LINUX and C)
        #endif	
#endif

//! Structure holding the parameters of the rover.
#ifndef ROVER_PARAM
EXPORT typedef struct rover_param
{
	//! number of wheels of the rover
	int WheelNumber;			

	//! position of walking wheels (1 -> walk, 0 -> no walk)
	int IsWalkingWheel[NUM_WHEEL_ROVER_MAX];				
	//! position of steering wheels (1 -> steer, 0 -> no steer)
	int IsSteeringWheel[NUM_WHEEL_ROVER_MAX];				
	//! position of drivig wheels (1 -> drive, 0 -> no drive)
	int IsDrivingWheel[NUM_WHEEL_ROVER_MAX];				

	//! radius of each wheel of the rover, in [m]
	double WheelRadius[NUM_WHEEL_ROVER_MAX];				
	//! wheel cartesian coordinates in 2D, in [m] (X [0] and Y [1])  in the rover frame
	double WheelCoordCart[NUM_WHEEL_ROVER_MAX][2];			
	//! wheel polar coordinates, in [m] and [rad] (D [0] and ALPHA [1]) in the rover frame
	double WheelCoordPol[NUM_WHEEL_ROVER_MAX][2];			
	//! difference of height (Z axis) in [m] between wheel centre and walking joint centre
	double LegLengthV[NUM_WHEEL_ROVER_MAX];					
	//! cartesian coordinates of the walking joints, in [m] (X [0] and Y [1])  in the rover frame
	double WalkCoordCart[NUM_WHEEL_ROVER_MAX][2];			
	//! polar coordinates of the walking joints, in [m] and [rad] (D [0] and ALPHA [1]) in the rover frame
	double WalkCoordPol[NUM_WHEEL_ROVER_MAX][2];			
	
} ROVER_PARAM;
#endif


/* ------------------------ */
/* -- GenericAckermann   -- */
/* ------------------------ */
/*!
	DEF.	:

		Compute a generic ackermann manoeuvre (can be a generic ackermann or a spot turn; not a straight line)
		The GenericAckermann function computes the steering angle and the rotation speed for each wheel of the rover.
		The input is the linear velocity, the coordinates of the center of rotation, and the point to control.
		The coordinates are expressed in the Cartesian rover frame. The point to control is a point that will describe
		a circle around the center of rotation. If the point to control is (0;0), the geometric center of the rover will
		turn around the center of rotation. This feature is useful to control the trajectory of a particular sensor. 
		The velocity is the linear velocity of the point to control, but if the center of rotation is the point to control,
		it is the angular velocity of the rover, in [rad/s].

	INPUTS	:
		- RoverVelocity			in [m/s] or [rad/s]
		- RotationCentre		in [m][m]
		- RoverPointToControl	in [m][m]

	OUTPUTS	:
		- WheelSteering			in [rad]
		- WheelVelocity			in [rad/s]
*/

//! Function to compute the wheel steering and the wheel velocity of the rover for a generic ackermann steering.
EXPORT int GenericAckermann( ROVER_PARAM *MyRover,	//!< ROVER_PARAM structure containing all the parameters of the rover.
	double RoverVelocity,							/*!< 
														 Rover generic velocity of the ackermann manoeuvre. <BR>
													     Can be a angular velocity in [rad/s] in case of a spot turn,
														 or a linear velocity in [m/s] in case of a generic turn
													*/
	double *RotationCenter,							//!< Cartesian coordinates of the centre of rotation for the ackermann.
	double *RoverPointToControl,					//!< Coordinates of the point to control of the rover, in the rover frame.
	double *WheelSteering,							//!< Angle of the steering wheels, in [rad].
	double *WheelVelocity							//!< Rotation velocity of the wheels, in [rad/s].
	);


/* ------------------------ */
/* -- Crab               -- */
/* ------------------------ */
/*!
	DEF.	:

		The Crab function computes the steering angle and the rotation speed for each wheel of the rover. 
		The steering is the same for each wheel (comes directly from the input HeadingAngle), 
		and the rotation speed depends only on the wheel radius.

		The function returns -1 if one of the input pointers is NULL, and 0 otherwise.

	INPUTS	:
		- RoverLinearVelocity	in [m/s]
		- HeadingAngle			in [rad]

	OUTPUTS	:
		- WheelSteering			in [rad]
		- WheelVelocity			in [rad/s]

*/

//! Function to compute the wheel steering and wheel velocity for a crab manoeuvre (i.e. the rover keeps the same orientation).
EXPORT int Crab( ROVER_PARAM *MyRover,	//!< ROVER_PARAM structure containing all the parameters of the rover.
	double RoverLinearVelocity,			//!< The linear velocity of the rover, in [m/s]
	double HeadingAngle,				//!< The heading angle of the motion, with respect to rover frame, in [rad]
	double *WheelSteering,				//!< Angle of the steering wheels, in [rad].			
	double *WheelVelocity 				//!< Rotation velocity of the wheels, in [rad/s].
	);


/* ------------------------ */
/* -- Low Cog            -- */
/* ------------------------ */
/*!	DEF.	:
		
		Compute the walking joint velocity and the wheel velocity when the rover lower its centre of gravity.
		During this manoeuvre, a wheel velocity is applied in order to keep the same contact point between the
		wheel and the ground.

	INPUTS	:
		- BodyZVelocity		in [m/s]
		- BodyHeightDispl	in [m]

	OUTPUTS	:
		- WalkVelocity		in [rad/s]
		- WheelVelocity		in [rad/s]
*/

//! Function to compute the walking joint velocity and the wheel velocity when the rover lower its centre of gravity.
EXPORT int LowCog( ROVER_PARAM *MyRover,	//!< ROVER_PARAM structure containing all the parameters of the rover.
	double BodyZVelocity,					//!< Rover vertical velocity for the motion, in [m/s].
	double BodyHeightDispl,					//!< Vertical total shift of the rover with respect to the position 
											//!< at the beginning of the manoeuvre, in [m].
	double *WalkVelocity,					//!< Rotation velocity of the wheel walking joints, in [rad/s].
	double *WheelVelocity					//!< Rotation velocity of the wheels, in [rad/s].
	);

#endif


/* ------------------------ */
/* -- SkidTurn           -- */
/* ------------------------ */
/*!
	DEF.	:
		
		The SkidTurn function computes the angular velocity of the wheel. In this implementation, only the distance 
		along the y axis from the wheel to the center of the osculating circle is taken into account. I.e. if all 
		the wheels from one side of the rover are aligned, one supposes that the wheels act as a tracked rover. The center 
		of the circle is on the y axis.
		For positive linear velocities and positive radius of curvature, the rover turns left. 
		For positive linear velocities and negative radius of curvature, the rover turns right.
		There are 3 different types of manoeuvre:
			- Straight line motion <br>
			  In this case, the input IsStraightLine is TRUE, and the parameter RadiusOfCurvature is ignored.
			- Spot turn motion <br>
			  When RadiusOfCurvature is 0. The rover turns on spot and the RoverVelocity represents the angular
			  rover velocity in [rad/s]. If RoverVelocity is positive, the rover turns left, and the rover turns 
			  right otherwise.
			- Skid turn motion
			  When RadiusOfCurvature is not equal to 0. The rover performs a generic skid turn.
			  The function returns -1 if one of the input pointers is NULL, and 0 otherwise.

	INPUTS	:
		- RoverVelocity			in [m/s] or [rad/s]
		- RadiusOfCurvature		in [m]
		- IsStraightLine		[-]

	OUTPUT	:
		- WheelVelocity			in [rad/s]

*/

//! Function to compute the wheel velocity for a skid turn (i.e. no steering of the wheels, only differential speed).
EXPORT int SkidTurn( ROVER_PARAM *MyRover,	//!< ROVER_PARAM structure containing all the parameters of the rover.
	double RoverVelocity,					/*!< 
												 Rover generic velocity of the skid turn manoeuvre. <BR>
											     Can be a angular velocity in [rad/s] in case of a spot turn
												 (i.e. RadiusOfCurvature = 0 ), or a linear velocity in [m/s] 
												 in case of a generic turn
											*/
	double RadiusOfCurvature,				//!< Radius of curvature of the manoeuvre
	int IsSkidPointTurn,					//!< Define if the manoeuvre is a point turn (1 -> true, 0 -> false)
	double *WheelVelocity					//!< Rotation velocity of the wheels, in [rad/s].
	);


/* ------------------------ */
/* -- SpotTurn           -- */
/* ------------------------ */
/*!
	DEF.	:
		
		The SpotTurn function computes the steering angle and angular velocity of each wheel of the rover.
		It takes into account which wheel is able to steer and to drive. <br>
		The function returns -1 if one of the input pointers is NULL, and 0 otherwise.

	INPUT	:
		- RoverAngularVelocity		in [rad/s]

	OUTPUTS	:
		- WheelSteering				in [rad]
		- WheelVelocity				in [rad/s]
*/

//! Function to compute the wheel steering and wheel velocity for a turn on spot.
EXPORT int SpotTurn( ROVER_PARAM *MyRover,	//!< ROVER_PARAM structure containing all the parameters of the rover.
	double RoverAngularVelocity,			//!< Rover angular velocity for the manoeuvre, in [rad/s]
	double *WheelSteering,	 				//!< Angle of the steering wheels, in [rad].
	double *WheelVelocity 	 				//!< Rotation velocity of the wheels, in [rad/s].
	);


/* ------------------------ */
/* -- Stop               -- */
/* ------------------------ */
/*!
	DEF.	:

		The Stop function sets all the wheel speed to 0.

	INPUT	:
		- no input

	OUTPUT	:
		- WheelVelocity		in [rad/s]
*/

//! Function to stop the rover.
EXPORT int Stop( ROVER_PARAM *MyRover,		//!< ROVER_PARAM structure containing all the parameters of the rover.
	double *WheelVelocity					//!< Rotation velocity of the wheels, in [rad/s].
	);


/* ------------------------ */
/* -- WheelWalk          -- */
/* ------------------------ */
/*!
	DEF.	:

		NOT IMPLEMENTED!
		
		Compute the wheel velocity and walking joint velocity with respect to the linear velocity of the manoeuvre.


	INPUTS	:
		- Distance			in [m]
		- LinearVelocity	in [m]
		- StepLength		in [m]
		- Gait				in [-]

	OUTPUTS	:
		- WalkVelocity		in [rad/s]
		- WheelVelocity		in [rad/s]
*/

//! NOT IMPLEMENTED! Function to compute the walking joint velocity and the wheel velocity for the wheel walking.
//EXPORT int WheelWalk( ROVER_PARAM *MyRover,		//!< ROVER_PARAM structure containing all the parameters of the rover.
//	double Distance,							//!< distance to walk, in [m]
//	double LinearVelocity,						//!< linear velocity, in [m/s]
//	double StepLength,							//!< length of each walk step, in [m]
//	int Gait,									/*!< kind of gait
//														- 0	->	IDX_GAIT_WBW, wheel by wheel
//														- 1	->	IDX_GAIT_ABA, axle by axle
//														- 2	->	IDX_GAIT_ZIG, zig-zag
//														- 3	->	IDX_GAIT_SBS, side by side
//														- 4	->	IDX_GAIT_ANT, ant
//												*/
//	double *WalkVelocity,						//!< Angle of the steering wheels, in [rad/s].
//	double *WheelVelocity						//!< Rotation velocity of the wheels, in [rad/s].
//	);
EXPORT int WheelWalk( ROVER_PARAM *MyRover,	
	double *StepLength,		
	int Gait,				
	double *WalkAngleRad,	
	double *WheelAngleRad	
	);



/* ------------------------ */
/* -- ConvCoordCart2Pol  -- */
/* ------------------------ */
/*!
	DEF.	:
		
		This function converts the set of cartesian coordinates given as input into polar coordinates.
		The format of the input and output is a table of 2 elements (X,Y coordinates or 
		RHO, THETA coordinates).

	INPUT	:
		- CoordCart		in [m][m]
		  Table of cartesian coordinates (X and Y).

	OUTPUTS	:
		- CoordPol		in [m][rad]
		  Table of corresponding polar coordinates (RHO, i.e. distance and THETA, i.e. angle ).
*/

//! Function to convert a set of cartesian coordinates into polar coordinates.
EXPORT void ConvCoordCart2Pol( const double CoordCart[][2],	
	double CoordPol[][2],
	int NumCoord
	);


/* ------------------------ */
/* -- ConvCoordPol2Cart  -- */
/* ------------------------ */
/*!
	DEF.	:
		
		This function converts the set of polar coordinates given as input into cartesian coordinates.
		The format of the input and output is a table of 2 elements (X,Y coordinates or 
		RHO, THETA coordinates).

	INPUT	:
		- CoordPol		in [m][rad]
		  Table of polar coordinates (RHO, i.e. distance and THETA, i.e. angle ).

	OUTPUTS	:
		- CoordCart		in [m][m]
		  Table of corresponding cartesian coordinates (X and Y).
*/

//! Function to convert a set of polar coordinates into cartesian coordinates
EXPORT void ConvCoordPol2Cart( const double CoordPol[][2],	
	double CoordCart[][2],
	int NumCoord
	);


/* ------------------------ */
/* -- RoverInit          -- */
/* ------------------------ */
/*!
	DEF.	:

		Initialize the ROVER_PARAM structure with a vector of parameters.
		For wheel cartesian coordinates : X, Y in [m]
		For wheel polar coordinates     : D [m] and ALPHA [rad]

	INPUT	:
		- MyRoverParameters
		  Table containing all the parameters for the ROVER_PARAM structure initialization.
				-	[0]			:			number of wheel
				-	[ 1]...[ 6]	:			position of walking wheels (1 -> walk, 0 -> no walk)
				-	[ 7]...[12]	:			position of steering wheels (1 -> steer, 0 -> no steer)
				-	[13]...[18]	:			position of drivig wheels (1 -> drive, 0 -> no drive)
				-	[19]...[24]	:			radius of the wheels in [m]
				-	[25]		:			indicate type of coordinates given
											(POLAR, CARTESIAN, BOTH)
				-	[26],[27]	:			wheel coordinate FL
				-	[28],[29]	:			FR
				-	[30],[31]	:			CL
				-	[32],[33]	:			CR
				-	[34],[35]	:			RL
				-	[36],[37]	:			RR
					ONLY IF TYPE OF COORDINATE IS BOTH :
					in this case, coord [26] to [37] are cartesian, and [38] to [49] are polar
				-	[38],[39]	:			wheel coordinate FL
				-	[40],[41]	:			FR
				-	[42],[43]	:			CL
				-	[44],[45]	:			CR
				-	[46],[47]	:			RL
				-	[48],[49]	:			RR				
				-	[38]...[43]	| B [50]...[55] :			length of the legs in [m]
				-	[44]		| B [56]		:			indicate type of coordinates given for walk position
															(POLAR, CARTESIAN, BOTH)
				-	[45],[46]	| B [57],[58]   :			walk coordinate FL
				-	[47],[48]	| B [59],[60]   :			FR
				-	[49],[50]	| B [61],[62]   :			CL
				-	[51],[52]	| B [63],[64]   :			CR
				-	[53],[54]	| B [65],[66]   :			RL
				-	[55],[56]	| B [67],[68]   :			RR
					ONLY IF TYPE OF COORDINATE IS BOTH :
					in this case, coord [45] to [56] are cartesian, and [57] to [68] are polar
				-	[57],[58]	| B [69],[70]   :			 walk coordinate FL
				-	[59],[60]	| B [71],[72]   :			 FR
				-	[61],[62]	| B [73],[74]   :			 CL
				-	[63],[64]	| B [75],[76]   :			 CR
				-	[65],[66]	| B [77],[78]   :			 RL
				-	[67],[68]	| B [79],[80]   :			 RR

	OUTPUT	:
		- MyRover
		  ROVER_PARAM structure initialized according to the parameters of the input
*/

//! Function to initialize the structure containing the parameters of the rover.
EXPORT int RoverInit( ROVER_PARAM *MyRover,		//!< ROVER_PARAM structure containing all the parameters of the rover.
	const double *MyRoverParameters,			//!< Table containing all the parameters needed for the initialization
	int NumParamIn								//!< Number of input parameters (initialization parameters, size of the table)
	);	//( ROVER_PARAM RoverConstants, double Inputs[12] );


/* ------------------------ */
/* -- RoverPrint         -- */
/* ------------------------ */
/*!
	DEF.	:

		Display on the standard output the ROVER_PARAM structure.

	INPUT	:
		- RoverConstants
		  ROVER_PARAM structure containing the rover parameters to display.

	OUTPUT	:
		- no output
*/

//! Function to display the parameters of the rover.
EXPORT int RoverPrint( ROVER_PARAM *RoverConstants );	//!< ROVER_PARAM structure containing all the parameters of the rover.

/* ------------------------ */
/* -- RoverReset         -- */
/* ------------------------ */
/*!
	DEF.	:

		Reset the parameters of the ROVER_PARAM structure. The function modifies the structure passed as input.

	INPUT	:
		- RoverConstants
		  ROVER_PARAM structure containing the rover parameters to reset.

	OUTPUT	:
		- no output
*/

//! Function to reset the parameters of the rover.
EXPORT int RoverReset( ROVER_PARAM *RoverConstants );	//!< ROVER_PARAM structure containing all the parameters of the rover.


/* ------------------------ */
/* -- Dist2              -- */
/* ------------------------ */
/*!
	DEF.	:
		
		Computes the distance between two points

	INPUTS	:
		- x1, y1
		  Coordinates of the first point.
		- x2, y2
		  Coordinates of the second point.

	OUTPUT	:
		Distance between the two points as return value.
*/

//! Function to compute the distance between two points given by their cartesian coordinates.
double Dist2( double x1, double y1,
	double x2, double y2 );
