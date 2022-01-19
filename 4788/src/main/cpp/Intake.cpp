#include "Intake.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

Intake::Intake(wml::actuators::DoubleSolenoid &leftIntakeActuation, wml::actuators::DoubleSolenoid &rightIntakeActuation, wml::TalonSrx &intakeMotor, SmartControllerGroup &contGroup) : _leftIntakeActuation(leftIntakeActuation),_rightIntakeActuation(rightIntakeActuation), _intakeMotor(intakeMotor), _contGroup(contGroup) {

}

void Intake::teleopOnUpdate(double dt) {
	double intakeManualSpeed = fabs(_contGroup.Get(ControlMap::IntakeManualSpin)) > ControlMap::TriggerDeadzone ? _contGroup.Get(ControlMap::IntakeManualSpin) : 0;

	_intakeMotor.Set(intakeManualSpeed);
}

//add in 2 pistons 