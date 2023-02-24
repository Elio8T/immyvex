using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftMotorFront;
extern motor RightMotorFront;
extern motor LeftMotorMiddle;
extern motor RightMotorMiddle;
extern motor LeftMotorBack;
extern motor RightMotorBack;
extern inertial Inertial;
extern digital_out Expansion;
extern controller Controller1;
extern motor IntakeMotor;
extern motor Shooter;
extern digital_out Blocker;
extern motor_group LeftM;
extern motor_group RightM;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );