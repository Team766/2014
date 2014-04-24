#include <types/vxTypes.h>

static const double WinchSpeed = -0.9;
static const double ArmWheelSpeed = 1;
static const double PassSpeed = 1;
static const double LimArmWheelSpeed = 0.75;
static const double PickupSpeed = 1;

static const double OutPower = -LimArmWheelSpeed;
static const double OutTime = 0.2;
static const double InPower = 1;
static const double InTime = 1;

//Arm Roller
static const float RollerInSpeed = 1;
static const float RollerOutSpeed = -1;
static const float RollerOffSpeed = 0;

//Eject Piston
static const bool EjectorOut = true;
static const bool EjectorIn = false;

//Arm Piston
static const bool ArmUp = false;
static const bool ArmDown = true;
static const bool GuardsIn = false;
static const bool GuardsOut = true;

static const bool GoaliePoleDown = false;
static const bool GoaliePoleUp = true;

//Catapult
static const bool CatapultLocked = false;
static const bool CatapultUnlocked = true;
static const double WaitAfterFire = 0.75;  //1.0

//Timers
static const int CatapultTimer = 1;

//AUTON

static const double LeftForPower = 0.5;
static const double RightForPower = 0.5;
static const double AutonTime = 2.0;
//static const double OneBallDistance = -0.6096;  // Two feet
//static const double TwoBallDistance = 0.0;  // Two feet
static const double DriveDistanceFireZone = 0.0;  //Any Auton that fires  -3.175
static const double Kp = 10.0;  //leave alone
static const double Kd = 0.8;  //  leave alone
static const double driveTolerance = .01;
static const double kDriveDistance2MovetoGetBall = -1.2; //0.9144 Two Ball
static const double kDriveDistance1P2 = -2.0;
static const double kDriveDistanceMoveAuton = -2.0; 
static const double ArmAfterFireWait = 2.0;
static const double ArmWaitforArm = 2.0;
static const double MoveForwardDistance = 0.0;
static const double WaitBeforeFiring = 0.0;
static const double kDriveDistance2MoveForwardtoGetBall = -2.6;
static const double OneBallMoveToFire = -1.2;
