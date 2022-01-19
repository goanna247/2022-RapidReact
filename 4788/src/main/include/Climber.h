#pragma once 

#include "controllers/Controllers.h"
#include "RobotMap.h"

class Climber {
 public:
	Climber(wml::actuators::DoubleSolenoid &leftClimber, wml::actuators::DoubleSolenoid &rightClimber, wml::controllers::SmartControllerGroup &contGroup);

	void teleopOnUpdate(double dt);
 private:
	wml::controllers::SmartControllerGroup &_contGroup;
	wml::actuators::DoubleSolenoid &_leftClimber;
	wml::actuators::DoubleSolenoid &_rightClimber;
};