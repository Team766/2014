//Brett's Code 2014

#define _USE_MATH_DEFINES
#include "WPILib.h"
#include "Ports.h"
#include "ButtonsAxes.h"
#include "RobotValues.h"
#include "CheezyDrive.h"
#include <math.h>
#include "subsystem/Intake.h"
#include "subsystem/Catapult.h"

SmartDashboard *dash;

class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system - placeholder
	Joystick stickLeft;
	Joystick stickRight;
	Joystick j3;
	
	Relay Compr;
	
	Talon LeftDrive;
	Talon RightDrive;
	Jaguar Winch;
	Talon ArmWheels;
	
	DigitalInput Presr;
	DigitalInput LauncherBotm1;
	DigitalInput LauncherBotm2;
	
	Solenoid Shifter;
	Solenoid WinchPist;
	//Solenoid WinchPistO;
	Solenoid Arm;
	//Solenoid ArmO;
	Solenoid BallGuard;
	Solenoid Ejector;
	
	Encoder LDriveEnc;
	Encoder RDriveEnc;
	
	CheezyDrive chezyDrive;
	Intake IntakeArm;
	Catapult Shooter;
	
	bool ArmC;
	bool shootbutton;

//	ofstream m_log;

public:
	RobotDemo(void):
		myRobot(6, 7),	//just have this in unused ports to make the rest work
		stickLeft(1),
		stickRight(2),
		j3(3),
		Compr(Relay_Compr),
		
		//PWM lines
		LeftDrive(PWM_Left_Drive),
		RightDrive(PWM_Right_Drive),
		Winch(PWM_Winch),
		ArmWheels(PWM_ArmWheels),
		
		// Digital 
		Presr(DIO_Pressure),
		LauncherBotm1(DIO_LauncherBotm1),
		LauncherBotm2(DIO_LauncherBotm2),
		//ShooterS(DIO_ShooterS),
		
		// Solenoids
		Shifter(Sol_Shifter),
		WinchPist(Sol_WinchPist),
		//WinchPistO(Sol_WinchPistO),
		Arm(Sol_Arm),
	    //ArmO(Sol_ArmO),
	    BallGuard(Sol_BallGuard),
	    Ejector(Sol_Ejector),
	    
	    //Encoders
	    LDriveEnc(DIO_LEncA, DIO_LEncB),
	    RDriveEnc(DIO_REncA, DIO_REncB),
		chezyDrive()
	{
		printf("2014 simple\n");
		myRobot.SetExpiration(0.1);
		GetWatchdog().Feed();
		dash->init();
		
		dash->PutNumber("RDriveDist", 0);
		dash->PutNumber("LDriveDist", 0);
		LDriveEnc.Start();
		RDriveEnc.Start();
		
		dash->PutNumber("DriveDistance1Ball", OneBallDistance);
		dash->PutNumber("Kp1Ball", OneBallKp);
		dash->PutNumber("Kd1Ball", OneBallKd);
		
/*		dash->PutNumber("ArmWheelSpeed",ArmWheelSpeed);
		dash->PutNumber("LimArmWheelSpeed",LimArmWheelSpeed);
		dash->PutNumber("WinchSpeed",WinchSpeed);
		dash->PutNumber("LeftForPower",LeftForPower);
		dash->PutNumber("RightForPower",RightForPower);
		dash->PutNumber("AutonTime",AutonTime);*/
		ArmC = true;
		shootbutton = false;
	}
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		GetWatchdog().Feed();
		
		if(Button_AutonSwitch){
			if(j3.GetRawButton(7)){
				LDriveEnc.Reset();
				RDriveEnc.Reset();
				//Move forward 24 inches
				const double kDriveDistance = dash->GetNumber("DriveDistance1Ball");
				const double Kp = dash->GetNumber("Kp1Ball"); //proportional constant  Real Number in robotvalues
				const double Kd = dash->GetNumber("Kd1Ball");;  //derivative constant           "   "
				double last_error = 0.0;
				while (IsAutonomous() && IsEnabled()) {
					const double error = kDriveDistance - (left_distance() + right_distance()) / 2.0;
					const double drive_power = Kp * error + Kd * (error - last_error) * 100.0;
					LeftDrive.SetSpeed(-drive_power);
					RightDrive.SetSpeed(drive_power);
					Wait(0.02);
					printf("error %f drive_power %f ld %f rd %f\n", error, drive_power, left_distance(), right_distance());
					last_error = error;
					if (error == 0) break;  //if in right spot, go to firing
				}
				//fire
				Shooter.update(true,
						 	   LauncherBotm1.Get());
				Winch.SetSpeed(Shooter.get_winch());
				WinchPist.Set(Shooter.get_winchLock());				
				
			}
				else if(j3.GetRawButton(8)){
					//two ball code here
				}
				else if(j3.GetRawButton(9)){
					LDriveEnc.Reset();
					RDriveEnc.Reset();
					const double kDriveDistance = -3.15;
					const double Kp = 10.0; //proportional constant 
					const double Kd = 0.8;  //derivative constant
					double last_error = 0.0;
					while (IsAutonomous() && IsEnabled()) {
						const double error = kDriveDistance - (left_distance() + right_distance()) / 2.0;
						const double drive_power = Kp * error + Kd * (error - last_error) * 100.0;
						LeftDrive.SetSpeed(-drive_power);
						RightDrive.SetSpeed(drive_power);
						Wait(0.02);
						printf("error %f drive_power %f ld %f rd %f\n", error, drive_power, left_distance(), right_distance());
						last_error = error;
					}
				}
				else if(j3.GetRawButton(10)){
					 // straigt and turn
				}
		}
	}
	/*int switch1 = j3.GetRawAxis(5);
	  int switch2 = j3.GetRawAxis(6);
			
			if(Button_AutonSwitch){
				switch(switch1){
						case 1:
							//run one ball auton
							break;
						case -1:
							//run two ball auton
							break;
				}
				switch(switch2){			
						case 1:
							//run drive straight
							break;
						case -1:
							// launch drive straight and turn auton
							break;
				}
			}
			*/
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(false);
		while (IsOperatorControl())
		{
			//Drivebase comment
			float LeftDriveC = -stickLeft.GetY();
			float RightDriveC = -stickRight.GetX();
			bool isQuickTurn = stickRight.GetRawButton(2);
			
			bool ShifterC = (stickLeft.GetRawButton(Button_Shifter));
			Shifter.Set(ShifterC);
			
			//Cheezy Drive (Left and Right Drive Control)
			chezyDrive.update(LeftDriveC, RightDriveC, isQuickTurn, !ShifterC);		
			LeftDrive.Set(chezyDrive.get_l());
			RightDrive.Set(chezyDrive.get_r());
			
			//Intake Arm Controls
			IntakeArm.update(j3.GetRawButton(Button_Ejector), 
							 j3.GetRawButton(Button_Inbound), 
							 j3.GetRawButton(Button_Pickup) ||
							 stickRight.GetRawButton(Button_DriverPickup));
			Ejector.Set(IntakeArm.get_ejector());
			ArmWheels.SetSpeed(IntakeArm.get_roller());
			Arm.Set(IntakeArm.get_armpist());
			//ArmO.Set(!IntakeArm.get_armpist());  ///Remove for Competion robot
			
			//Catapult Controls
			Shooter.update(stickRight.GetRawButton(Button_DriverShoot) ||
					       j3.GetRawButton(Button_ShooterLaunch),
					       LauncherBotm1.Get() ||
					       j3.GetRawButton(Button_Cancel));
			Winch.SetSpeed(Shooter.get_winch());
			WinchPist.Set(Shooter.get_winchLock());
			
			//Compressor
			Compr.Set(Presr.Get()? Relay::kOff : Relay::kForward);
			
			GetWatchdog().Feed();
			Wait(0.005);
			
		}
	}

	float left_distance() {
		return translateDrive(LDriveEnc.GetRaw());
	}
	float right_distance() {
		return translateDrive(RDriveEnc.GetRaw());
	}
	
	float translateDrive(float trans){
		float wheel_d = 0.0899;
		float counts = 256 * 4.0;
		return (trans / counts) * (atan(1) * 4) * wheel_d;
	}
};

START_ROBOT_CLASS(RobotDemo);

