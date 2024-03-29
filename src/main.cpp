#include "main.h"
#include "init.h"
#include "pid.h"
#include "math.h"


void allWheelsMoveSteady(int power) {
	int heading = Inertial.get_heading();

	if (Inertial.get_heading() > heading) {
		leftWheels.move(power);
		rightWheels.move(power * 0.95);
	}
	else if (Inertial.get_heading() < heading) {
		leftWheels.move(power * 0.95);
		rightWheels.move(power);
	} else {
		allWheels.move(power);
	}
}

void PIDMover(
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
	double kP = 0.9;
	double kI = 0;
	double kD = 0.25;

// Checks if the movement is positive or negative
	bool isPositive = setPoint > 0;

// PID LOOPING VARIABLES
	setPoint = setPoint * 2.54; // converts from inches to cm

	double wheelCircumference = 3.14 * 2.75; // 4 is the wheel diameter in inches
	double gearRatio = 1;
	double wheelRevolution = wheelCircumference * 2.54; // in cm
	long double singleDegree = wheelRevolution / 360;

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

	Master.print(0, 0, "CMR: %f", currentMotorReading);

	double currentDistanceMovedByWheel = 0;

	prevError = (int) (setPoint - currentDistanceMovedByWheel);

	int timeout = 0;

	while (!actionCompleted) {
	// PID CALCULATION CODE
		
	// P: Proportional -- slows down as we reach our target for more accuracy
	
		// error = goal reading - current reading
		error = int (setPoint - currentDistanceMovedByWheel);
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = int (integral + error);
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
        derivative = int (error - prevError);
		// sets the previous error to the previous error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;

		power = proportionalOut + integralOut + derivativeOut;



	// PID LOOPING CODE

		allWheelsMoveSteady(power);

		pros::delay(15);

		br = backRight.get_position();
		bl = backLeft.get_position();
		fr = frontRight.get_position();
		fl = frontLeft.get_position();

		currentMotorReading = ((br + bl + fr + fl) / 4); // degrees
		currentWheelReading = currentMotorReading / gearRatio; // degrees = degrees * multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		if ((((currentDistanceMovedByWheel >= setPoint) && (setPoint > 0)) || ((currentDistanceMovedByWheel <= setPoint) && (setPoint < 0))) || 
			((power <= 10) && (power >= -10)) ||
			(((currentDistanceMovedByWheel >= (setPoint - 20)) && (setPoint > 0)) || ((currentDistanceMovedByWheel <= (setPoint + 20)) && (setPoint < 0)))) {
				actionCompleted = true;
				allWheels.brake();
		}
	}
}

void PIDTurner(
		int setPoint, // how far you want to move in inches
		int direction // 1 for left and 2 for right
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
	double kP = 0.9;
	double kI = 0;
	double kD = 0.3;

// Checks if the movement is positive or negative
	bool isPositive = (setPoint - setPoint) > 0;

// PID LOOPING VARIABLES
	int negativePower;

	int inertialReadingInit = Inertial.get_heading();
	int distanceToMove;

	if (direction == 1) {
		// standard left turn is negative, so the calculation makes it positive if it is a normal turn
		// ex: current = 90, goal = 45 -> -45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn left
		distanceToMove = inertialReadingInit - setPoint;
	}
	else if (direction == 2) {
		// standard right turn is positive, so the calculation keeps it positive if it is a normal turn
		// ex: current = 45, goal = 90 -> 45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn right
		distanceToMove = setPoint - inertialReadingInit;
	}

	// if the error is positive, then the calculation is fine and is left
	if (distanceToMove >= 0) {
		// do nothing
	}
	// otherwise, the turn takes the "long way" around the circle, and the calculation has provided the
	// value of the negative short way - adding 360 to this value gives the long way around the circle,
	// which is what is needed
	// ex: current = 90, goal = 45, direction = right -> calculated -45 degree turn -> + 360 -> 315 (length of long way)
	// 45 - 90 = -45 (short way, negative) + 360 = 315 (long way, positive)
	else {
		distanceToMove += 360;
	}
	// the calculation has now yielded a positive value that is the error needed of our turn in the proper
	// direction, making it similar to how a forward/backward movement is coded

	// finally, the code sets a new value that will be set to the distance moved to zero to finalize this similarity
	// distanceToMove is analogous to setPoint on PIDMover, and changeInReading is analogous to currentDistanceMovedByWheel
	int changeInReading = 0;

	prevError = (int) (distanceToMove - changeInReading);

	Master.print(0, 0, "DTM: %d", distanceToMove);

	int timeout = 0;

	while (!actionCompleted) {
	// PID CALCULATION CODE
		
	// P: Proportional -- slows down as we reach our target for more accuracy
	
		// error = goal reading - current reading
		error = distanceToMove - changeInReading;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = int (integral + error);
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
        derivative = int (error - prevError);
		// sets the previous error to the previous error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;

		power = proportionalOut + integralOut + derivativeOut;



	// PID LOOPING CODE

		negativePower = power * -1;

		// the power will never be negative and invert the turns because distanceToMove is always positive
		if (direction == 1) {
			leftWheels.move(negativePower);
			rightWheels.move(power);
		}
		else if (direction == 2) {
			leftWheels.move(power);
			rightWheels.move(negativePower);
		}

		pros::delay(15);

		// the change in reading is set to the absolute value of the change in reading due to everything being positive
		int changeInDistance = direction == 1 
			? inertialReadingInit - Inertial.get_heading() 
			: Inertial.get_heading() - inertialReadingInit;
		changeInReading = changeInDistance < 0
		    ? changeInDistance + 360
			: changeInDistance;

		// int exampleVar = Inertial.get_heading() - inertialReadingInit;
		// changeInReading = std::abs(Inertial.get_heading() - inertialReadingInit);

		if ((changeInReading >= distanceToMove) || 
			((power <= 10) && (power >= -10)) ||
			((changeInReading <= (distanceToMove + 10)) && (changeInReading >= (distanceToMove - 10)))) {
				actionCompleted = true;
				allWheels.brake();
		}
	}
}






































/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "1166T - Technically Legal");
	pros::lcd::set_text(1, "RICO HAS RETURNED!");
	pros::lcd::set_text(2, "RICO HAS RETURNED!");
	pros::lcd::set_text(3, "RICO HAS RETURNED!");
	pros::lcd::set_text(4, "RICO HAS RETURNED!");
	pros::lcd::set_text(5, "RICO HAS RETURNED!");
	pros::lcd::set_text(6, "RICO HAS RETURNED!");
	pros::lcd::set_text(7, "RICO HAS RETURNED!");
	pros::lcd::set_text(8, "RICO HAS RETURNED!");
	pros::lcd::set_text(9, "RICO HAS RETURNED!");

	Inertial.tare_heading();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	Shield.set_value(false);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	Shield.set_value(false);
	autonnumber = 1;
	if (autonnumber == 1) {
		// Near Autonomous

		// Move to center of field, intake Triball, and move back most of the way
		Intake.move(-128);
		PIDMover(47);
		pros::delay(500);

		PIDMover(-35);

		Intake.brake();

		// Move to the MLZ bar
		leftWheels.move(64);
		rightWheels.move(-64);
		waitUntil((Inertial.get_heading() > 5) && (Inertial.get_heading() < 15));
		allWheels.brake();
		PIDMover(-26);

		// Angle with the MLZ bar and scoop the Triball out
		PIDTurner(130, 2);
		plowBackRight.set_value(true);
		pros::delay(200);

		// Move down the straightaway, bringing all Triballs with us
		leftWheels.move(-64);
		rightWheels.move(64);
		waitUntil((Inertial.get_heading() > 15) && (Inertial.get_heading() < 35));
		allWheels.brake();
		plowBackRight.set_value(false);
		pros::delay(1000);
		PIDTurner(100, 2);


		Intake.move(128);
		pros::delay(200);
		PIDMover(40);
		Intake.brake();

		PIDMover(-1.5);
	}
	else if (autonnumber == 2) {
		// Far Autonomous

		// Bringing down the intake
		allWheels.move(-128);
		pros::delay(100);
		allWheels.brake();

		pros::delay(250);

		

		// Intake the Triball in front of the robot and move back
		Intake.move(-128);
		PIDMover(30);
		PIDMover(-37);

		// Turn and scoop the Triball in the MLZ, then turn toward the goal
		leftWheels.move(-32);
		rightWheels.move(32);
		waitUntil((Inertial.get_heading() > 320) && (Inertial.get_heading() < 340));
		allWheels.brake();
			
		plowBackLeft.set_value(true);
		
		PIDMover(-22);
		// PIDTurn();
		plowBackLeft.set_value(false);

		leftWheels.move(-64);
		rightWheels.move(64);
		waitUntil((Inertial.get_heading() > 300) && (Inertial.get_heading() < 320));
		allWheels.brake();





		allWheels.move(-128);
		pros::delay(1000);





		for(int i=0;i<3;i++){

			allWheels.move(128);
			pros::delay(200);
			allWheels.move(-128);
			pros::delay(500);
		}


		PIDMover(16);

		PIDTurner(90,2);

		Intake.move(128);

		allWheels.move(128);
		pros::delay(1000);
		allWheels.brake();
		Intake.brake();

		allWheels.move(-128);
		pros::delay(500);
		allWheels.brake();
		
	}
	else if (autonnumber == 4) {
		// Loser Far Autonomous
		allWheels.move(128);
		pros::delay(1000);
		allWheels.brake();
	}else if (autonnumber == 5) {
		// Loser Far Autonomous
	/*
		allWheels.move(128);
		pros::delay(1000);
		allWheels.brake();
	*/
		PIDMover(20);
		
	}else if (autonnumber == 3) {
		// Autonomous Skills

		allWheels.set_brake_modes(MOTOR_BRAKE_BRAKE);
		
		// Line up with MLZ bar
		allWheels.move(-64);
		waitUntil((Inertial.get_heading() > 90) && (Inertial.get_heading() < 350));
		allWheels.brake();

		// Shooting triballs
		plowBackRight.set_value(true);
		Indexer.move(-128);
		//pros::delay(1000);
		pros::delay(32000);
		Indexer.brake();
		plowBackRight.set_value(false);

		// Turning and moving down the straightaway
		leftWheels.move(64);
		rightWheels.move(-64);
		waitUntil(Inertial.get_heading() > 350);
		allWheels.brake();
		Intake.move(128);
		PIDMover(83);
		Master.print(0, 0, "QK");






		//turns and drives to the middle of the field
		PIDTurner(260,1);
		PIDMover(24);
		leftWheels.move(-64);
		rightWheels.move(64);
		waitUntil((Inertial.get_heading() > 220) && (Inertial.get_heading() < 235));
		allWheels.brake();
		PIDMover(27);

		//turns parrallel the Barrier
		leftWheels.move(64);
		rightWheels.move(-64);
		waitUntil((Inertial.get_heading() > 240) && (Inertial.get_heading() < 275));
		allWheels.brake();

		//drive to line up for the first push
		PIDMover(25);

		//turns for the first push
		leftWheels.move(-64);
		rightWheels.move(64);
		waitUntil((Inertial.get_heading() > 187) && (Inertial.get_heading() < 213));
		allWheels.brake();
		pros::delay(250);

		//sets plows down and drives to goal
		plowBackRight.set_value(true);
		allWheels.move(-128);
		pros::delay(1500);
		allWheels.brake();

		//back away from the goal & plows up
		plowBackRight.set_value(false);
		PIDMover(34);
		

		//turns parrallel the Barrier
		leftWheels.move(32);
		rightWheels.move(-32);
		waitUntil((Inertial.get_heading() > 255) && (Inertial.get_heading() < 265));
		allWheels.brake();

		//moves along the Barrier
		//PIDMover(40);

			allWheels.move_relative((4*3.14*110),100);

			pros::delay(1500);


			allWheels.brake();
			pros::delay(500);

		//turns for the second push
		leftWheels.move(-64);
		rightWheels.move(64);
		waitUntil((Inertial.get_heading() > 195) && (Inertial.get_heading() < 205));
		allWheels.brake();

		pros::delay(200);

		leftWheels.move(32);
		rightWheels.move(-32);
		waitUntil((Inertial.get_heading() > 215) && (Inertial.get_heading() < 225));
		allWheels.brake();

		pros::delay(200);

		//does the second push
		plowBackRight.set_value(true);
		plowBackLeft.set_value(true);
		allWheels.move(-128);
		pros::delay(2000);
		allWheels.brake();

		//drive away from the goal
		PIDMover(14);
		plowBackRight.set_value(false);
		plowBackLeft.set_value(false);

		//turn to edge on field
		leftWheels.move(-32);
		rightWheels.move(32);
		waitUntil((Inertial.get_heading() > 45) && (Inertial.get_heading() < 94));
		allWheels.brake();

		pros::delay(200);

		//drive to edge of field
		PIDMover(53);

		//turn to do final push
		leftWheels.move(64);
		rightWheels.move(-64);
		waitUntil((Inertial.get_heading() > 140) && (Inertial.get_heading() < 180));
		allWheels.brake();

		pros::delay(200);

		//do final push
		rightWheels.move(-70);
		leftWheels.move(-128);
		pros::delay(3000);

		for(int i=0;i<3;i++){

			allWheels.move(128);
			pros::delay(300);
			allWheels.move(-128);
			pros::delay(600);

		}
		allWheels.move(128);
		pros::delay(200);
		allWheels.brake();
/*
		leftWheels.move(-32);
		rightWheels.move(32);
		waitUntil((Inertial.get_heading() > 180) && (Inertial.get_heading() < 185));
		allWheels.brake();
		plowBackLeft.set_value(true);
		plowBackRight.set_value(true);
		allWheels.move(-128);
		pros::delay(3000);
		allWheels.brake();
		Intake.brake();



/*
		// Ram into goal, back up, ram into goal again, then back up again
		// rSide full, lSide 90% for 2 seconds
		// back up for 1/2 second
		// drive forward 1 second
		// back up WITH PID 
		leftWheels.move(-64);
		rightWheels.move(32);
		waitUntil((Inertial.get_heading() > 325) && (Inertial.get_heading() < 345));
		plowFrontLeft.set_value(true);
		allWheels.brake();
		pros::delay(200);
		leftWheels.move(80);
		rightWheels.move(128);

	  //Checks to see if we have moved into the wall or not
		while (!(Inertial.get_heading() > 270) && (Inertial.get_heading() < 280))
		{
			if((Inertial.get_heading() > 355) || (Inertial.get_heading() < 5)){
				PIDTurner(275, 1);
			}
		}

		pros::delay(1250);

		leftWheels.move(128);
		rightWheels.move(128);
		pros::delay(750);
		allWheels.brake();

		leftWheels.move(-128);
		rightWheels.move(-128);
		pros::delay(250);
		allWheels.brake();

		plowFrontLeft.set_value(false);
		PIDTurner(120,1);

		leftWheels.move(-128);
		rightWheels.move(-128);
		pros::delay(500);
		allWheels.brake();
		
		PIDMover(-7);
*/
/*
		// Turn to center of field, then move there
		plowFrontLeft.set_value(false);
		PIDTurner(210,1);
		PIDMover(48);


		// Turn to face goal and move there
		//PIDTurner(anotherValue);
		PIDTurner(170,1);
		plowBackLeft.set_value(true);
		plowBackRight.set_value(true);
		allWheels.brake();
		pros::delay(250);


		allWheels.move(-128);
		pros::delay(750);

		// Move back
		PIDMover(33);

		// Turn 90 degrees and move to the other side of the field there
		leftWheels.move(64);
		rightWheels.move(-64);
		waitUntil((Inertial.get_heading() > 260) && (Inertial.get_heading() < 275));
		allWheels.brake();
		pros::delay(100);
		PIDMover(16);

		// Turn toward goal and move there
		leftWheels.move(-64);
		rightWheels.move(64);
		waitUntil((Inertial.get_heading() > 355) || (Inertial.get_heading() < 5));
		allWheels.brake();
		PIDMover(33);
/*
		// Move back, turn toward other MLZ bar, and move there
		PIDMover(-45);
		PIDTurner(valU);
		PIDMover(50);
		
		// Turn toward other side of goal and move there
		PIDTurner(valueBillion);
		PIDMover(40);

		// End of routine
	*/
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	while (true) {

	//Drivetrain
    	drvtrFB = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    	drvtrLR = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if((abs(drvtrFB) > drvtrDZ) || (abs(drvtrLR) > drvtrDZ)) {
      		// ^^ Checks to see if either joystick has moved out of the deadzone
      		rightWheels.move((drvtrFB-(drvtrLR)));
      		leftWheels.move((drvtrFB+(drvtrLR)));
    	} else {
      		rightWheels.brake();
      		leftWheels.brake();
    	}  

	//Intake
		if(Master.get_digital(DIGITAL_RIGHT)==true){
			Intake.move(-128);
		} else if(Master.get_digital(DIGITAL_LEFT)==true){
			Intake.move(128);
		} else {
			Intake.brake();
		}

	//Indexer
		if(Master.get_digital(DIGITAL_Y)==true){
			indxrmove = true;
			Indexer.set_brake_mode(MOTOR_BRAKE_COAST);
			Indexer.move(-128);
		} else if(Master.get_digital(DIGITAL_A)==true){
			indxrmove = true;
			Indexer.set_brake_mode(MOTOR_BRAKE_COAST);
			Indexer.move(128);
		} else if(Master.get_digital(DIGITAL_X)==true){
			Indexer.set_brake_mode(MOTOR_BRAKE_HOLD);
			indxrmove = true;
			Indexer.move_relative(-1500,100);
			pros::delay(750);
			indxrmove = false;
		} else {
			Indexer.brake();
		}

		if ((Master.get_digital(DIGITAL_Y)==false)&&(Master.get_digital(DIGITAL_A)==false)){
			indxrmove = false;
		}
		
		if(indxrmove == false){
			Indexer.brake();
		}

	//Plows
	  //Front plows
		if((Master.get_digital(DIGITAL_L2))&&(pf==false)){
			plowFrontLeft.set_value(true);
			plowFrontRight.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_L2)==false);
			pf = true;
		}
		if((Master.get_digital(DIGITAL_L2))&&(pf==true)){
			plowFrontLeft.set_value(false);
			plowFrontRight.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_L2)==false);
			pf = false;
		}

		if((Master.get_digital(DIGITAL_R2))&&(pf==false)){
			plowFrontLeft.set_value(true);
			plowFrontRight.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_L2)==false);
			pf = true;
		}
		if((Master.get_digital(DIGITAL_R2))&&(pf==true)){
			plowFrontLeft.set_value(false);
			plowFrontRight.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_L2)==false);
			pf = false;
		}

	  //Back plows
		if(((Master.get_digital(DIGITAL_L1)==true)||(Master.get_digital(DIGITAL_R1)==true))&&(pb==false)){
			plowBackLeft.set_value(true);
			plowBackRight.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_L1)==false);
			pb = true;
		}
		if(((Master.get_digital(DIGITAL_L1)==true)||(Master.get_digital(DIGITAL_R1)==true))&&(pb==true)){
			plowBackLeft.set_value(false);
			plowBackRight.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_L1)==false);
			pb = false;
		}

	  //Shield
		if((Master.get_digital(DIGITAL_B)==true)&&(shieldRaised==false)){
			Shield.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_B)==false);
			shieldRaised = true;
		}
		if((Master.get_digital(DIGITAL_B)==true)&&(shieldRaised==true)){
			Shield.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_B)==false);
			shieldRaised = false;
		}
		pros::delay(20);
	}
}
