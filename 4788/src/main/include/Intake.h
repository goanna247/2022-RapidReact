#pragma once 

#include "controllers/Controllers.h"
#include "RobotMap.h"

class Intake {
 public:
	Intake(wml::actuators::DoubleSolenoid &leftIntakeActuation, wml::actuators:::DoubleSolenoid &rightIntakeActuation, wml::TalonSrx &intakeMotor, wml::controllers::SmartControllerGroup &contGroup);


	void teleopOnUpdate(double dt);

 private:
	wml::controllers::SmartControllerGroup &_contGroup;

	wml::actuators::DoubleSolenoid &_leftIntakeActuation;
	wml::actuators::DoubleSolenoid &_rightIntakeActuation;
	wml::TalonSrx &_intakeMotor;
};