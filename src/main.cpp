#include "main.h"
#include "init.h"

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
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void autonomous() {}

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
      		rightWheels.move((drvtrFB-drvtrLR));
      		leftWheels.move((drvtrFB+drvtrLR));
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
			Indexer.move(-128);
		} else if(Master.get_digital(DIGITAL_A)==true){
			Indexer.move(128);
		} else {
			Indexer.brake();
		}

	//Plows
	  //Left front plow
		if((Master.get_digital(DIGITAL_L2)==true)&&(pfl==false)){
			plowFrontLeft.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_L2)==false);
			pfl = true;
		}
		if((Master.get_digital(DIGITAL_L2)==true)&&(pfl==true)){
			plowFrontLeft.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_L2)==false);
			pfl = false;
		}

	  //Left back plow
		if((Master.get_digital(DIGITAL_L1)==true)&&(pbl==false)){
			plowBackLeft.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_L1)==false);
			pbl = true;
		}
		if((Master.get_digital(DIGITAL_L1)==true)&&(pbl==true)){
			plowBackLeft.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_L1)==false);
			pbl = false;
		}

	  //Right front plow
		if((Master.get_digital(DIGITAL_R2)==true)&&(pfr==false)){
			plowFrontRight.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_R2)==false);
			pfr = true;
		}
		if((Master.get_digital(DIGITAL_R2)==true)&&(pfr==true)){
			plowFrontRight.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_R2)==false);
			pfr = false;
		}

	  //Right back plow
		if((Master.get_digital(DIGITAL_R1)==true)&&(pbr==false)){
			plowBackRight.set_value(true);
			waitUntil(Master.get_digital(DIGITAL_R1)==false);
			pbr = true;
		}
		if((Master.get_digital(DIGITAL_R1)==true)&&(pbr==true)){
			plowBackRight.set_value(false);
			waitUntil(Master.get_digital(DIGITAL_R1)==false);
			pbr = false;
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

	  //
	  	//if(){

		//}
		pros::delay(20);
	}
}
