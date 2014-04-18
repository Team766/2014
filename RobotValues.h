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

//Catapult
static const bool CatapultLocked = false;
static const bool CatapultUnlocked = true;
static const double WaitAfterFire = 0.75;

//Timers
static const int CatapultTimer = 1;

//AUTON

static const double LeftForPower = 0.5;
static const double RightForPower = 0.5;
static const double AutonTime = 2;
static const double OneBallDistance = -0.6096;  // Two feet
static const double TwoBallDistance = -0.6096;  // Two feet
static const double Kp = 10.0;
static const double Kd = 0.8;
static const double driveTolerance = .005;
static const double kDriveDistance3 = 0.9144;
static const double kDriveDistance1P2 = -.5;
static const double kDriveDistanceMoveAuton = -1.0;



