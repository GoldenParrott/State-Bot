int PID(
		int currentReading, // the current sensor reading
		int setPoint, // the goal sensor reading
		int prevError, // the error from the previous cycle
		int moveOrTurn, // 1 for moving forward/backward, 2 for turning
		int sizeOfTurn // 1 for small and 2 for big (only used in turning)
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
		kI = 0.5;
		kD = 0.5;
	}
	else if (moveOrTurn == 2) {
		if (sizeOfTurn == 1) {
			kP = 1.6;
			kI = 0.4;
			kD = 0.25;
		}
		else if (sizeOfTurn == 2) {
			// P = 1.2, D = 4
			kP = 1.2;
			kI = 0.4;
			kD = 4;
		}
	}


	// gets the robot's initial reading on the sensor to detect whether it is positive or negative
	bool isPositive = currentReading > 0;
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

/*
void PIDMove(
	int goalReading // the distance to move (in inches)
	)
{
	goalReading = goalReading * 2.54; // converts from inches to cm

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
	double prevError = goalReading - prevDistance;

	while (!actionCompleted) {

		power = PID(currentDistanceMovedByWheel, goalReading, prevError, 1, 1);

		prevDistance = currentDistanceMovedByWheel;
		prevError = goalReading - prevDistance;

		allWheelsMoveSteady(power);

		pros::delay(15);

		br = backRight.get_position();
		bl = backLeft.get_position();
		fr = frontRight.get_position();
		fl = frontLeft.get_position();

		currentMotorReading = ((br + bl + fr + fl) / 4); // degrees
		currentWheelReading = currentMotorReading / gearRatio; // degrees = degrees * multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		if ((currentDistanceMovedByWheel == goalReading) || 
			((power <= 10) && (power >= -10)) ||
			((currentDistanceMovedByWheel <= (goalReading + 10)) && (currentDistanceMovedByWheel >= (goalReading - 10)))) {

			actionCompleted = true;
		}
	}

	Master.clear();
	
}

void PIDTurn(
	int goalReading, // the inertial heading to turn to
	int direction // 1 for left, 2 for right
	)
	// IF PID TURN PASSES 0 FROM ABOUT 180 DURING A TURN, THE NUMBER IT IS SET TO CAN BE NO LESS THAN 90
{
	bool actionCompleted = false;
	int power;
	int negativePower;

	int currentInertialReading = (int) Inertial.get_rotation();
	int inertialReadingInit = currentInertialReading;

	int prevReading = currentInertialReading;

	double prevError = goalReading - prevReading;

	bool isPositive;
	if (direction == 1) {isPositive = false;}
	else {isPositive = true;}

	

	while (!actionCompleted) {

		if ((goalReading - inertialReadingInit) <= 75) {
			power = PID(currentInertialReading, goalReading, prevError, 2, 1);
		}
		else {
			power = PID(currentInertialReading, goalReading, prevError, 2, 2);
		}

		negativePower = power * -1;

		
		if (((currentInertialReading > goalReading) && (isPositive)) ||
			((currentInertialReading < goalReading) && (!isPositive))) {
			if (power > 0) {
				power = power;
				negativePower = power * -1;
			}
			else if (power < 0) {
				negativePower = power;
				power = power * -1;
			}
		}
		if (power > 0) {
			power = power;
			negativePower = power * -1;
		}
		else if (power < 0) {
			negativePower = power;
			power = power * -1;
		}
		

		prevReading = currentInertialReading;
		prevError = goalReading - prevReading;

		if (direction == 1) {
			leftWheels.move(negativePower);
			rightWheels.move(power);
		}
		else if (direction == 2) {
			leftWheels.move(power);
			rightWheels.move(negativePower);
		}

		pros::delay(15);

		Master.print(0, 0, "Inertial Rotation: %f", currentInertialReading);

		currentInertialReading = (int) Inertial.get_rotation();

		if ((currentInertialReading == goalReading) || 
			((power <= 0.5) && (power >= -0.5))) { // ||
			// (currentInertialReading < goalReading + 5) && (currentInertialReading > goalReading - 5)) {

			actionCompleted = true;
		}

		Master.clear();
	}
}
*/