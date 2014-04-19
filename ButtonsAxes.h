#include <types/vxTypes.h>

//If the buttons are not labeled on the joysticks themselves, 
//a handy way to view them (on Windows) is 'joy.cpl' (enter in the start menu)
// and then select the joystick. It will show which buttons are which.

//driver joysticks

// 1: trigger
// 2-6: they are labeled
// 7: the little pinky button

static const uint32_t Button_Shifter = 1;
static const uint32_t Button_Halfspeed = 7;
static const uint32_t Button_Reverse = 2;
static const uint32_t Button_DriverShoot = 1;
static const uint32_t Button_DriverPickup = 7; //make toggle
static const uint32_t Button_QuickTurn = 2;



//Box joystick
/*
 * Box op is able to run on either the OI or the silver KOP joystick.
 * For the box op the buttons are as follows:
 * 
 * 1: red button labeled shoot
 * 2: switch below the red button
 * 3/5: switch labeled roller
 * 4: white button at left of triangle
 * 6: white button at top of triangle
 * 7: auton on/off switch
 * 11: white button at bottom right of triangle
 * 12: switch labled arm up/down
 * axis 4: the clamp switch - if less than 1 true
 * knob (runs through POV hat)
 *  slot 1: labeled forward
 *  slot 2: labeled 1 ball
 *  slot 3: labeled 2 ball
 *  slot 4: labeled 3 ball
 *  
 * For the silver KOP buttons are all labeled with numbers, except
 * trigger is 1 and thumb button is 2.
 * For knob emulation:
 *  slot 1: hold right
 *  slot 2: hold left
 *  slot 3: hold down
 *  slot 4: hold up
 */ 

static const uint32_t Axis_BallGuard = 4;

static const uint32_t Button_ShooterLaunch = 1;
static const uint32_t Button_Cancel = 2;
static const uint32_t Button_WinchOn = 2;
static const uint32_t Button_RollerIn = 3;
static const uint32_t Button_RollerOut = 5;
static const uint32_t Button_Pickup = 4;
static const uint32_t Button_Inbound = 6;
static const uint32_t Button_Ejector = 11;
static const uint32_t Button_ArmDown = 12;
static const uint32_t Button_GoaliePole = 10;

static const uint32_t Button_AutonSwitch = 7;
static const uint32_t Axis_Horizontal = 5;
static const uint32_t Axis_Verticle = 6;
