#ifndef CHEEZY_DRIVE_H
#define CHEEZY_DRIVE_H
#include "WPILib.h"
//#include "../CommonCSV.h"
class CheezyDrive
{
public:
	CheezyDrive();
	~CheezyDrive();
	void update(float throttle, float wheel, bool quickTurn, bool hiGear);
	void updateTank(float r_, float l_, bool quickTurn, bool hiGear);
	float get_r(); 
	float get_l();
private:
	float out_r; 
	float out_l;

	float old_wheel_;
	float quickStopAccumulator_;
};

#endif
