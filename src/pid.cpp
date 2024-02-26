// #include "init.h"
// #include "pid.h"
/*
int PIDMover(
		int setPoint // how far you want to move in inches
		)
{
// PID CALCULATION VARIABLES
// General Variables
	int error;
	int power;
	bool actionCompleted = false;

// Proportional Variables
	int proportionalOut;

// Integral Variables
	int integral;
	int integralLimiter;
	int integralOut;

// Derivative Variables
    int derivative;
    int derivativeOut;
	int prevError = error;

// Constants -- tuning depends on whether the robot is moving or turning
	double kP = 1;
	double kI = 0.5;
	double kD = 0.5;

// Checks if the movement is positive or negative
	bool isPositive = (setPoint - setPoint) > 0;

// PID LOOPING VARIABLES
	setPoint = setPoint * 2.54; // converts from inches to cm

	double wheelCircumference = 3.14 * 2.75; // 4 is the wheel diameter in inches
	double gearRatio = 1;
	double wheelRevolution = wheelCircumference * 2.54; // in cm
	long double singleDegree = wheelRevolution / 360;

	bool actionCompleted = false;
	int power;

	backRight.tare_position();
	backLeft.tare_position();
	frontRight.tare_position();
	frontLeft.tare_position();

	double br;
	double bl;
	double fr;
	double fl;

	double currentMotorReading = ((br + bl + fr + fl) / 4);
	double currentWheelReading = currentMotorReading * gearRatio;

	double currentDistanceMovedByWheel = 0;

	double prevDistance = currentDistanceMovedByWheel;
	double prevError = setPoint - prevDistance;

	while (!actionCompleted) {
	// PID CALCULATION CODE
		// P: Proportional -- slows down as we reach our target for more accuracy
	
		// error = goal reading - current reading
		error = setPoint - currentDistanceMovedByWheel;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




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
		// sets the previous error to the previous error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;

		power = proportionalOut + integralOut + derivativeOut;





	// PID LOOPING CODE

		prevDistance = currentDistanceMovedByWheel;
		prevError = setPoint - prevDistance;

		allWheelsMoveSteady(power);

		pros::delay(15);

		br = backRight.get_position();
		bl = backLeft.get_position();
		fr = frontRight.get_position();
		fl = frontLeft.get_position();

		currentMotorReading = ((br + bl + fr + fl) / 4); // degrees
		currentWheelReading = currentMotorReading / gearRatio; // degrees = degrees * multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		if ((currentDistanceMovedByWheel == setPoint) || 
			((power <= 10) && (power >= -10)) ||
			((currentDistanceMovedByWheel <= (setPoint + 10)) && (currentDistanceMovedByWheel >= (setPoint - 10)))) {
				actionCompleted = true;
		}
	}
}
*/