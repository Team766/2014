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
		
		//dash->PutNumber("RDriveDist", 0);
		//dash->PutNumber("LDriveDist", 0);
		LDriveEnc.Start();
		RDriveEnc.Start();
		
		dash->PutNumber("DriveDistance1Ball", OneBallDistance);
		dash->PutNumber("DriveDistance2Ball", TwoBallDistance);
		//dash->PutNumber("Kp1Ball", OneBallKp);
		//dash->PutNumber("Kd1Ball", OneBallKd);
		
		ArmC = true;
		shootbutton = false;
	}
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		GetWatchdog().Feed();
		
		//If Auton is Enabled
		if(j3.GetRawButton(Button_AutonSwitch)){
			printf("Auton Enabled \n");
			
			// Shoot One!!!
			if(j3.GetRawAxis(Axis_Horizontal) == 1){  //Fix command from sequential to concurent
				printf("One Ball Auton \n");
				
				//Move forward 24 inches
				const double kDriveDistance1 = dash->GetNumber("DriveDistance1Ball");
				autonDriveToDistance(kDriveDistance1);
				
				printf("Ready to shoot \n");
				
				//fire after it moves forward (stop when reached waitingToWinch in state machine)
				while(IsAutonomous() && IsEnabled() && !Shooter.waitingToWinch()){
					printf("In Shooter Loop \n");
					Shooter.update(true,
							       !LauncherBotm1.Get());
					Winch.SetSpeed(Shooter.get_winch());
					WinchPist.Set(Shooter.get_winchLock());
				}
				
				Wait(0.05);
				
				// Move forward again
				autonDriveToDistance(kDriveDistance1P2);
				
				printf("Ready to Winch \n");
				
				// Bring the shooter down
				while(IsAutonomous() && IsEnabled()){
					Shooter.update(false,
							!LauncherBotm1.Get());
					Winch.SetSpeed(Shooter.get_winch());
					WinchPist.Set(Shooter.get_winchLock());
				}
				
			}
			// Two Ball Auton
			else if(j3.GetRawAxis(Axis_Horizontal) == -1){
				printf("Two Ball Auton \n");
				// Shoot Two!!!
				//Move forward 24 inches
				const double kDriveDistance = dash->GetNumber("DriveDistance2Ball");
				autonDriveToDistance(kDriveDistance);

				printf("Ready to shoot \n");
								
					//fire after it moves forward (stop when reached waitingToWinch in state machine)
					while(IsAutonomous() && IsEnabled() && !Shooter.waitingToWinch()){
						printf("In Shooter Loop \n");
						Shooter.update(true,
									   !LauncherBotm1.Get());
						Winch.SetSpeed(Shooter.get_winch());
						WinchPist.Set(Shooter.get_winchLock());				
					}
								
					Wait(0.05);
					//winch down
					while(IsAutonomous() && IsEnabled()){
						Shooter.update(false,
						         	   !LauncherBotm1.Get());
						Winch.SetSpeed(Shooter.get_winch());
						WinchPist.Set(Shooter.get_winchLock());
						if (!LauncherBotm1.Get()) break; //waits till winch is stopped to exit loop
					}
					
								
					// Move backwards 3 meters and collect
					/*IntakeArm.update(false, 
									 false, 
									 true,
									 false,
									 false,
									 false);
					
					ArmWheels.SetSpeed(IntakeArm.get_roller());
					Arm.Set(IntakeArm.get_armpist());
					*/
					ArmWheels.SetSpeed(1);
					Arm.Set(true);
					
					autonDriveToDistance(kDriveDistance3); //move back 3 ft.
					
					ArmWheels.SetSpeed(0);
					Arm.Set(ArmUp);
								
					autonDriveToDistance(-kDriveDistance3); //move forward 3 ft.
					
					//fire
					while(IsAutonomous() && IsEnabled() && !Shooter.waitingToWinch()){
						printf("In Shooter Loop \n");
						Shooter.update(true,
									   !LauncherBotm1.Get());
						Winch.SetSpeed(Shooter.get_winch());
						WinchPist.Set(Shooter.get_winchLock());				
					}
													
					Wait(0.05);
					
					//forwards 2 meters to cross line for 5 extra points
					autonDriveToDistance(-3.0);
								
					// Bring the shooter down
					while(IsAutonomous() && IsEnabled()){
						Shooter.update(false,
						         	   !LauncherBotm1.Get());
						Winch.SetSpeed(Shooter.get_winch());
						WinchPist.Set(Shooter.get_winchLock());
					}
								
			}
			// Drive Forward Auton
			else if(j3.GetRawAxis(Axis_Verticle) == 1){
				printf("Drive Forward Auton \n");
				
				autonDriveToDistance(kDriveDistanceMoveAuton);
			}
			else if(j3.GetRawAxis(Axis_Verticle) == -1){
				printf("Brett hasn't told me auton \n");
				 // straigt and turn
			}
			else{
				printf("No Auton Selected \n");
			}
		}
	}

	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(false);
		while (IsOperatorControl())
		{
			//Drivebase
			float LeftDriveC = -stickLeft.GetY();
			float RightDriveC = -stickRight.GetX();
			bool ShifterC = (stickLeft.GetRawButton(Button_Shifter));
			Shifter.Set(ShifterC);
			
			//Cheezy Drive (Left and Right Drive Control)
			chezyDrive.update(LeftDriveC, 
							  RightDriveC, 
							  stickRight.GetRawButton(Button_QuickTurn), 
							  !ShifterC);		
			LeftDrive.Set(chezyDrive.get_l());
			RightDrive.Set(chezyDrive.get_r());
			
			//Intake Arm Controls
			IntakeArm.update(j3.GetRawButton(Button_Ejector), 
							 j3.GetRawButton(Button_Inbound), 
							 j3.GetRawButton(Button_Pickup) ||
							 stickRight.GetRawButton(Button_DriverPickup),
							 j3.GetRawButton(Button_RollerIn),
							 j3.GetRawButton(Button_RollerOut),
							 j3.GetRawButton(Button_ArmDown));
			Ejector.Set(IntakeArm.get_ejector());
			ArmWheels.SetSpeed(IntakeArm.get_roller());
			Arm.Set(IntakeArm.get_armpist());
			
			
			//Catapult Controls
			Shooter.update(stickRight.GetRawButton(Button_DriverShoot) ||
					       j3.GetRawButton(Button_ShooterLaunch),
					       !LauncherBotm1.Get() ||
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
	
	// TO METERS
	float translateDrive(float trans){
		float wheel_d = 0.0899;
		float counts = 256 * 4.0;
		return (trans / counts) * (atan(1) * 4) * wheel_d;
	}
	
	void autonDriveToDistance(float kDriveDistance){
		//Reseting Encoders
		LDriveEnc.Reset();
		RDriveEnc.Reset();
		//Move forward 24 inches
		//const double kDriveDistance = dash->GetNumber("DriveDistance1Ball");
		//const double Kp = 10.0; //proportional constant  Real Number in robotvalues
		//const double Kd = 0.8;  //derivative constant           "   "
		double last_error = 0.0;
		// Kp, Kd from RobotValues.h
		while (IsAutonomous() && IsEnabled()) {
			const double error = kDriveDistance - (left_distance() + right_distance()) / 2.0;
			const double drive_power = Kp * error + Kd * (error - last_error) * 100.0;
			LeftDrive.SetSpeed(-drive_power);
			RightDrive.SetSpeed(drive_power);
			Wait(0.02);
			//printf("error1 %f drive_power %f ld %f rd %f\n", error, drive_power, left_distance(), right_distance());
			last_error = error;
			// Figure out a nicer way to do this later
			if (fabs(error) <= (0 + driveTolerance)){
				LeftDrive.SetSpeed(0);
				RightDrive.SetSpeed(0);
				break;  //if in right spot, go to firing
			}
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

