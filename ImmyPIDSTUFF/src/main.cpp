/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Tue Oct 01 2019                                           */
/*    Description:  Accurate Turns                                            */
/*                  This program will use the Gyro to make a 90 degree turn.  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftMotorFront       motor         11              
// RightMotorFront      motor         1               
// LeftMotorMiddle      motor         12              
// RightMotorMiddle     motor         2               
// LeftMotorBack        motor         13              
// RightMotorBack       motor         3               
// Inertial             inertial      14              
// Expansion            digital_out   A               
// Controller1          controller                    
// IntakeMotor          motor         5               
// Shooter              motor         4               
// Blocker              digital_out   F               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
using namespace vex;

//Global instance of competition
competition Competition;

void pre_auton(void) {
  //Initializing something important or some shit fr fr 
  vexcodeInit();
}

void Intake(float value, int direction) {
  //1 will spin it forward, which is intaking
  //Anything else will make it go reverse to shoot
  if(direction == 1) {
    IntakeMotor.spin(vex::directionType::fwd, value, vex::velocityUnits::pct);
  } else {
    IntakeMotor.spin(vex::directionType::rev, value, vex::velocityUnits::pct);
  }
}






//VARIABLE DECLERATION
//Joystick Control
int deadband = 10;
float leftJoystickY = 0;
float rightJoystickX = 0;
const float MaxMotorVolatage = 15;



void StopLeftMotorGroup() {
  LeftMotorFront.stop();
  LeftMotorMiddle.stop();
  LeftMotorBack.stop();
}
void StopRightMotorGroup() {
  RightMotorFront.stop();
  RightMotorMiddle.stop();
  RightMotorBack.stop();
}
void SetLeftMotorGroupVelocity(int percentValue){
  LeftMotorFront.setVelocity(percentValue, percent);
  LeftMotorMiddle.setVelocity(percentValue, percent);
  LeftMotorBack.setVelocity(percentValue, percent);
}
void SetRightMotorGroupVelocity(int percentValue){
  RightMotorFront.setVelocity(percentValue, percent);
  RightMotorMiddle.setVelocity(percentValue, percent);
  RightMotorBack.setVelocity(percentValue, percent);
}
void SetLeftMotorSpinForward(){
  LeftMotorFront.spin(forward);
  LeftMotorMiddle.spin(forward);
  LeftMotorMiddle.spin(forward);
}
void SetRightMotorSpinForward(){
  RightMotorFront.spin(forward);
  RightMotorMiddle.spin(forward);
  RightMotorBack.spin(forward);
}

void DebugScreen(){
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Distance From Target:");
  //Brain.Screen.print(last_error);
  Brain.Screen.newLine();
  Brain.Screen.print("TargetHeading:");
  //Brain.Screen.print(target_angle);
  Brain.Screen.newLine();
  Brain.Screen.print("Heading:");
  Brain.Screen.print(Inertial.heading(degrees));
  Brain.Screen.newLine();
  Brain.Screen.print("PID_Out_Put:");
  //Brain.Screen.print(PID());
}


  


// _______________  PID CONTROL _______________________
//
//

//Bens pid stuff which I messed up sorry
  //double DeltaTime = 0;
 // double accumulation_of_error = 0;
  //double derivative_of_error = 0;
  //double last_error = 0;

  //PID variables
  //double current_angle = 0;
  //double target_angle = 0;
  //double integral = 0;
  //  double error = target_angle - current_angle;
 //  accumulation_of_error += error * DeltaTime;
  // derivative_of_error += (error - last_error)/DeltaTime;
 //  last_error = error;
 //   (error * Kp) + (accumulation_of_error * Ki) + (derivative_of_error * Kd);
  // }



//PID Settings
double Kp = 0.0; 
double Ki = 0.0;  
double Kd = 0.0;

double turnKp = 0.0; 
double turnKi = 0.0;  
double turnKd = 0.0;

//test variable
int desiredvalue = 200;
int desiredTurnvalue = 0;

//Lateral PID stuff
int error;  //Sensor value - desired value | positoin
int prevError = 0;  //position 20 ms ago
int derivative; //error - prevError | speed
int totalError = 0;

//Turning PID stuff
int turnError;  //Sensor value - desired value | positoin
int turnPrevError = 0;  //position 20 ms ago
int turnDerivative; //error - prevError | speed
int turnTotalError = 0;

bool resetDriveSensors = false;





//Is only on during autonomous
bool enableDrivePID = true;




//Actual function
int drivePID() {
  while(enableDrivePID) {
    


    if(resetDriveSensors) {
      resetDriveSensors = false;
      LeftMotorFront.setPosition(0,degrees);
      LeftMotorMiddle.setPosition(0,degrees);
      LeftMotorBack.setPosition(0,degrees);

      RightMotorFront.setPosition(0,degrees);
      RightMotorMiddle.setPosition(0,degrees);
      RightMotorBack.setPosition(0,degrees);


    }

    //Get position of both motors
    int leftMotorPostion = LeftMotorMiddle.position(degrees);
    int rightMotorPostion = RightMotorMiddle.position(degrees);
    ///////////////////////////////////////
    //__________________Lateral PID__________________
    ////////////////////////////////////////////////////
    //Get the average of both
    int averagePosition = (leftMotorPostion + rightMotorPostion)/2;

    //potential
    error = averagePosition - desiredvalue;

    //Derivative
    derivative = error - prevError;

    //Integral
    //totalError += error;

    double lateralMotorPower = (error * Kp + derivative + Kd);


    ///////////////////////////////////////
    //__________________Turning PID__________________
    ////////////////////////////////////////////////////
    int turnDifference = (leftMotorPostion - rightMotorPostion);

    //potential
    turnError = turnDifference - desiredTurnvalue;

    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral
    //turnTotalError += turnError;

    double turnMotorPower = (turnError * turnKp + turnDerivative + turnKd);




    LeftMotorFront.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LeftMotorMiddle.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LeftMotorBack.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);

    RightMotorFront.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    RightMotorMiddle.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    RightMotorBack.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);


    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);
  }


  return 1;
}
//
//
// ____________ END OF PID CONTROL ____________________





//PUT AUTON SHIT IN HERE _______________________________
void soloAWP() {
  //turn, go forward, blah blah blah
  //desiredvalue = 300;
  //desiredturnValue = 200;
}

void autonomous(void) {

  


  resetDriveSensors = true;
  desiredvalue = 300;
  desiredTurnvalue = 200;
  vex::task::sleep(1000);

  resetDriveSensors = true;
  desiredvalue = 300;
}

//End of Auton stuff ___________________________________



//Put User Control stuff here ________________________
void usercontrol(void) {
  
  //Affects impact of the turning
  double turnImportance = 0.5;
  
  
  enableDrivePID = false;
  // Wait time between cycles of the main loop (ms)
  int cycleTime = 20;
  
  
  LeftMotorFront.setBrake(brakeType::coast);
  LeftMotorMiddle.setBrake(brakeType::coast);
  LeftMotorMiddle.setBrake(brakeType::coast);
  RightMotorFront.setBrake(brakeType::coast);
  RightMotorMiddle.setBrake(brakeType::coast);
  RightMotorBack.setBrake(brakeType::coast);
  Shooter.setBrake(brakeType::brake);
  Expansion.set(false);
  Blocker.set(false);

  //Main Loop
  while(1) {
    

    //old stuff
    //leftJoystickY = (Controller1.Axis3.position()/100) * MaxMotorVolatage;

    // Get the velocity percentage of the right motor. (Axis2)
    //rightJoystickY = (Controller1.Axis2.position()/100) * MaxMotorVolatage;

    leftJoystickY = (Controller1.Axis3.position(percent));
    rightJoystickX = (Controller1.Axis1.position(percent));
    if(leftJoystickY<deadband){
      leftJoystickY = 0;
    }
    if(rightJoystickX<deadband){
      rightJoystickX = 0;
    }

    double turnVolts = rightJoystickX * 0.12;
    //double forwardVolts = leftJoystickY * 0.12 * (1- (std::abs(turnVolts)/12.0) * turnImportance);
    double forwardVolts = leftJoystickY * 0.12;

    if (std::abs(leftJoystickY) < deadband||std::abs(rightJoystickX) < deadband) {
      // Set the speed to zero.
      //SetLeftMotorGroupVelocity(0);
      
      LeftM.stop();
      RightM.stop();

    } else {
      // Set the speed to leftMotorSpeed
      //SetLeftMotorGroupVelocity(leftJoystickY);
      LeftM.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
      RightM.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

  

    }

    


    // Spin both motors in the forward direction.
    //SetLeftMotorSpinForward();
    //SetRightMotorSpinForward();

    //Shooting Sequence
    if (Controller1.ButtonX.pressing()) {
			Shooter.spin(forward, 100.0, volt);
      wait(0.9, seconds);
      IntakeMotor.spin(reverse, 100.0, volt);
      wait(2,seconds);
      Shooter.stop();
      IntakeMotor.stop();
		}

    //Intake Disc
    while (Controller1.ButtonR1.pressing()) {
			IntakeMotor.spin(forward, 100.0, volt);
    } 

    if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()) {
			IntakeMotor.stop();
    } 

    while (Controller1.ButtonR2.pressing()) {
			IntakeMotor.spin(reverse, 100.0, volt);
    } 
    if (Controller1.ButtonDown.pressing()) {
			Expansion.set(true);
      Blocker.set(true);
    } 

    



    wait(cycleTime, msec);
  }


}












int main() {
  double startOfFrame;
  double endOfFrame;

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate the Gyro, and wait for it to finish.
  Inertial.calibrate();
  waitUntil(!Inertial.isCalibrating());


  // Set the Gyro's heading to 0.
  Inertial.setHeading(180, degrees);

  //Reset Brain Timer
  Brain.Timer.reset();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();



  while (true) {
    startOfFrame = Brain.Timer.value();

  

    wait(25, msec);
    endOfFrame = Brain.Timer.value();

    //Debug Values to Screen
    DebugScreen();

    //calculate time between frames
    //DeltaTime = startOfFrame - endOfFrame;
  }    
}
