#pragma once 

#include "controllers/Controllers.h"
#include "RobotMap.h"

class Climber {
 public:
	Climber(wml::controllers::SmartControllerGroup &contGroup);

	void teleopOnUpdate(double dt);
 private:
	wml::controllers::SmartControllerGroup &_contGroup;
};