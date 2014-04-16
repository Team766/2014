#ifndef INTAKE_H
#define INTAKE_H

#include "WPILib.h"

class Intake {
public:
	void update(bool button_eject, bool button_inbound, bool button_intake);
	void manual(bool button_in, bool button_out, bool button_arm);
	//void intakeToggle(bool button_driverIntake);
	bool get_ejector(void);
	float get_roller(void);
	bool get_armpist(void);
private:
	bool out_ejector;
	float out_roller;
	bool out_armpist;
};

#endif
