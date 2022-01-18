#pragma once 

#include "RobotMap.h"

class Drivetrain : public wml::Strategy {
 public:
	Drivetrain(std::string name, wml::Drivetrain &drivetrain, wml::controllers::SmartControllerGroup &contGroup);

	void OnUpdate(double dt);
 private:
  wml::Drivetrain &_drivetrain;
	wml::controllers::SmartControllerGroup &_contGroup;

	double _leftPower, _rightPower;
};