#include "main.h"
#include "pid.h"

int PID(
		int currentReading, // the current sensor reading
		int setPoint, // the goal sensor reading
		int prevError, // the error from the previous cycle
		int moveOrTurn // 1 for moving forward/backward, 2 for turning
		)
{
// General Variables
	int error;
	int power;

// Proportional Variables
	int proportionalOut;

// Integral Variables
	int integral;
	int integralLimiter;
	int integralOut;

// Derivative Variables
    int derivative;
    int derivativeOut;

// Constants -- tuning depends on whether the robot is moving or turning
	double kP;
	double kI;
	double kD;
	if (moveOrTurn == 1) {
		kP = 1;
		kI = 0.25;
		kD = 0.25;
	}
	else if (moveOrTurn == 2) {
		kP = 0.75;
		kI = 0.25;
		kD = 0.1;
	}


	// gets the robot's initial reading on the sensor to detect whether it is positive or negative
	bool isPositive;
	if (currentReading > 0) {
		isPositive = true;
	} else if (currentReading < 0) {
		isPositive = false;
	}
	// P: Proportional -- slows down as we reach our target for more accuracy
	
		// error = goal reading - current reading
		error = setPoint - currentReading;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = (error * kP);


	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = integral + error;
		// prevents the integral variable from causing the robot to overshoot
		if ((isPositive && (error <= 0)) || (!isPositive && (error >= 0))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		if (error >= integralLimiter) {
            integral = 0;
        }
		// kI (integral constant) brings integral down to a reasonable/useful output number
		integralOut = integral * kI;


	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
		// the error from the previous cycle should be taken as a parameter
        derivative = error - prevError;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;


    // Output Power -- the actual output power

        power = proportionalOut + integralOut + derivativeOut;
    
    // the movement itself
        return power;
}