#ifndef CATAPULT_H
#define CATAPULT_H

#include "WPILib.h"

class Catapult {
public:
	Catapult() : state_(WINCHING), out_winch(0.0), out_winchLock(false) {}
	void update(bool button_shoot, bool cancel);
	float get_winch(void);
	bool get_winchLock(void);
	bool waitingToWinch(void);
private:
	enum State {
		WINCHING,
		FIRING,
		READY_TO_FIRE,
		WAITING_TO_WINCH
	};
	State state_;
	float out_winch;
	bool out_winchLock;
	Timer shootTimer;
};

#endif
/*
class Catapult {
public:
	Catapult(Jaguar * winch, Solenoid * dogGear, DigitalInput * limitSwitch);
	void wantShoot(bool shoot);
	void wantReload();
	void wantAutoShoot(bool autoShoot);
private:
	void setWinch(bool winchOn);
	void setPiston();
	void setReleasePiston(bool shoot);
	bool isCatapultDown();

	Talon * winch;
	Solenoid * dogGear;
	DigitalInput * limitSwitch;
};
*/
