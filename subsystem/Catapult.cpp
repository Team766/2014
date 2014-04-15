#include "Catapult.h"
#include "../RobotValues.h"


void Catapult::update(bool button_shoot, bool cancel) {
	bool winchLockC = CatapultLocked;
	float winchC = 0;
	switch (state_) {
		case WINCHING:
			winchLockC = CatapultLocked;
			if (cancel) {
				winchC = 0;
				state_ = READY_TO_FIRE;
				printf("Stopping!\n");
			} else {
				winchC = WinchSpeed;
			}
			break;
		case FIRING:
			winchLockC = CatapultUnlocked;
			winchC = 0;
			state_ = WAITING_TO_WINCH;
			printf("Firing\n");
			shootTimer.Reset();
			shootTimer.Start();
			break;
		case READY_TO_FIRE:
			winchC = 0;
			winchLockC = CatapultLocked;
			if (button_shoot) {
				state_ = FIRING;
				printf("Now firing\n");
			}
			break;
		case WAITING_TO_WINCH:
			winchLockC = CatapultUnlocked;
			winchC = 0;
			if(shootTimer.Get() >= WaitAfterFire){
				state_ = WINCHING;
				printf("Now winching\n");
			}
			break;
	}

	out_winch = winchC;
	out_winchLock = winchLockC;	
}
float Catapult::get_winch(void)  {
	return out_winch;
}
bool Catapult::get_winchLock(void)  {
	return out_winchLock;
}


/*
Catapult::Catapult(Jaguar *winch, Solenoid* dogGear, DigitalInput* limitSwitch) {
	this->winch = winch;
	this->dogGear = dogGear;
	this->limitSwitch = limitSwitch;
}

void Catapult::wantShoot(bool shoot) {
	setReleasePiston(shoot);	
}
void Catapult::wantReload() {
	setReleasePiston(false);
	while(!limitSwitch->Get()) {
		setWinch(true);
	}
	setWinch(false);
}
void Catapult::wantAutoShoot(bool autoShoot){
	Catapult::wantShoot(true);
	//probably wait a second here or whatever
	wantReload();
}
void Catapult::setWinch(bool winchOn) {
	double power;
	if(winchOn) {
		power = 1.0;
	} else {
		power = 0;
	}
	winch->Set(power);
}

void Catapult::setReleasePiston(bool shoot) {
	if(shoot)
		dogGear->Set(true);
	else 
		dogGear->Set(false);
}
*/
