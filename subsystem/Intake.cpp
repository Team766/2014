#include "Intake.h"
#include "../RobotValues.h"

void Intake::update(bool button_eject, bool button_inbound, bool button_intake)  {
	
	// piston in
	bool ejectorC = EjectorIn;
	// roller off
	float rollerC = RollerOffSpeed;
	// arm up
	bool armpistC = ArmUp;
	
	if(button_eject){
		printf("Eject \n");
		//piston out
		ejectorC = EjectorOut;
		//Roller Out
		rollerC = RollerOutSpeed;
	}
	if(button_inbound){
		//Arm Down
		armpistC = ArmDown;
		//roller out
		rollerC = RollerOutSpeed;
	}
	if(button_intake){
		//Arm Down
		armpistC = ArmDown;
		//Roller In
		rollerC = RollerInSpeed;
	}
	
	out_ejector = ejectorC;
	out_roller = rollerC;
	out_armpist = armpistC;
}
void Intake::manual(bool button_in, bool button_out, bool button_arm){
	float rollerC = RollerOffSpeed;
	bool armpistC = ArmUp;
	
	if(button_in) rollerC = RollerInSpeed;
	if(button_out) rollerC = RollerOutSpeed;
	if(button_arm) armpistC = ArmDown;
	
	out_roller = rollerC;
	out_armpist = armpistC;
}
bool Intake::get_ejector(void)  {
	return out_ejector;
}
float Intake::get_roller(void)  {
	return out_roller;
}
bool Intake::get_armpist(void)  {
	return out_armpist;
}









/*
Swag State Macchine(254)

Intake::update()  {
	newState = state;
	switch(state) {
		
		 * intake -> roller_on WHILE button_pressed -> default
		 * outbound -> exhaust_with_arm_down -> move arm down(?) -> roller_reversed -> default
		 * exhaust_with_arm_up -> roller_reversed -> default
		 * default -- sets motor to 0, set arm to default state 
		 
	case MANUAL:
		intake.set(0);
		setArmDown(false);
		setEjector(false);
		if(wantIntake) {
			newState = INTAKE;
		} else if(wantInbound){
			newState = INBOUND;
		} else if(wantExhaust) {
			newState = EXHAUST;
		}
		break;
	case INTAKE:
		intake.set(1);
		setArmDown(true);
		if(!wantIntake) 
			newState = MANAUL;
		break;
	
	case EXHAUST:
		intake.set(-1);
		setEjector(true);
		if(!wantExhaust)
			newState = MANAUL;
		break;
		
	case INBOUND:
		setArmDown(true);
		intake.set(-1);
		if(!wantInboud)
			newState = MANAUL;
		break;
		
	case SWAG:
		break;
	}
	
	if(state != newState) {
		state = newState;
	}
}
*/
