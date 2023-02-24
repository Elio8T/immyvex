#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftMotorFront = motor(PORT11, ratio18_1, true);
motor RightMotorFront = motor(PORT1, ratio18_1, false);
motor LeftMotorMiddle = motor(PORT12, ratio18_1, true);
motor RightMotorMiddle = motor(PORT2, ratio18_1, false);
motor LeftMotorBack = motor(PORT13, ratio18_1, true);
motor RightMotorBack = motor(PORT3, ratio18_1, false);
motor_group LeftM (LeftMotorFront,LeftMotorMiddle,LeftMotorBack);
motor_group RightM (RightMotorFront,RightMotorMiddle,RightMotorBack);
inertial Inertial = inertial(PORT14);
digital_out Expansion = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);
motor IntakeMotor = motor(PORT5, ratio18_1, false);
motor Shooter = motor(PORT4, ratio6_1, false);
digital_out Blocker = digital_out(Brain.ThreeWirePort.F);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}