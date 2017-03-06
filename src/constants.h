// constants.h

#ifndef CONSTANTS_H
#define CONSTANTS_H

//#define DEBUG		//!< Constant to define the debug mode. If not defined, the functions will not print anything.


#ifndef NUM_WHEEL_ROVER_MAX
	#define NUM_WHEEL_ROVER_MAX 6	//!< Constant of the maximal number of wheels that a rover can have 
									//!< (for coding purpose, i.e. initialization of the size of the tables)
#endif

// constants for the index of the wheel in a table
#define W1 0	//!< Constant defining the index of the first wheel in a table.
#define W2 1	//!< Constant defining the index of the second wheel in a table.
#define W3 2	//!< Constant defining the index of the third wheel in a table.
#define W4 3	//!< Constant defining the index of the forth wheel in a table.
#define W5 4	//!< Constant defining the index of the fifth wheel in a table.
#define W6 5	//!< Constant defining the index of the sixth wheel in a table.

// constants for the index of the walking joint in a table
#define WJ1 0	//!< Constant defining the index of the first walking joint in a table.
#define WJ2 1	//!< Constant defining the index of the second walking joint in a table.
#define WJ3 2	//!< Constant defining the index of the third walking joint in a table.
#define WJ4 3	//!< Constant defining the index of the forth walking joint in a table.
#define WJ5 4	//!< Constant defining the index of the fifth walking joint in a table.
#define WJ6 5	//!< Constant defining the index of the sixth walking joint in a table.

/* ---  Constants ---  */
#ifndef M_PI
	#define M_PI 3.14159265358979323846		//!< Mathematical pi constant.
#endif 
#ifndef RAD2DEG
	#define RAD2DEG	180./M_PI				//!< Mathematical conversion factor from radians to degrees.
#endif
#ifndef DEG2RAD
	#define DEG2RAD M_PI/180.				//!< Mathematical conversion factor from degrees to radians.
#endif

#ifndef EPSILON
	#define EPSILON .00001					//!< Minimal difference for double comparisons.
#endif

#ifndef TRUE
	#define TRUE 1							//!< Define logical true value (1).
#endif

#ifndef FALSE
	#define FALSE 0							//!< Define logical false value (0).	
#endif

// constants for the gait for the wheel walking
#define	IDX_GAIT_WBW	0		//!< Gait index (wheel walking) for wheel by wheel.
#define	IDX_GAIT_ABA	1		//!< Gait index (wheel walking) for axle by axle.
#define	IDX_GAIT_ZIG	2		//!< Gait index (wheel walking) for zig-zag.
#define	IDX_GAIT_SBS	3		//!< Gait index (wheel walking) for side by side.
#define	IDX_GAIT_ANT	4		//!< Gait index (wheel walking) for ant.

// constants for the type of coordinates
#define CARTESIAN 0		//!< Constant defining the cartesian type of coordinates (RoverInit).
#define POLAR 1			//!< Constant defining the polar type of coordinates (RoverInit).
#define BOTH 2			//!< Constant defining both types of coordinates;  WARNING firt CARTESIAN, then POLAR.

// constant to determine which coordinate is X and Y in a table
#define X 0				//!< Constant defining the X cartesian coordinates.
#define Y 1				//!< Constant defining the Y cartesian coordinates.
// constant of the index of the polar coordinates
#define D 0		 		//!< Constant defining the D (distance) polar coordinates.
#define ALPHA 1	 		//!< Constant defining the ALPHA (angle) polar coordinates.

// constants for unit testing
#define EQUAL 1			//!< constants used for the unit testing
#define GREATER 2		//!< constants used for the unit testing
#define LOWER 0			//!< constants used for the unit testing
#define DEFAULT -1		//!< constants used for the unit testing
#define SUCCESS 1		//!< constants used for the unit testing
#define FAILURE -1		//!< constants used for the unit testing

/* --- Macros  --- */
#ifndef ABS
	#define ABS(x)	((x) < 0 ? -(x) : (x))		//!< Macro to compute the absolute value.
#endif

#ifndef SGN
	#define SGN(x)	((x) < 0 ? -1 : 1 )			//!< Macro to compute the sign value.
#endif


#endif
