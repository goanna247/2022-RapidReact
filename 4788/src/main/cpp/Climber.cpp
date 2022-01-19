#include "Climber.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

Climber::Climber(wml::actuators::DoubleSolenoid &leftClimber, wml::actuators::DoubleSolenoid &rightClimber, SmartControllerGroup &contGroup) : _leftClimber(leftClimber), _rightClimber(rightClimber), _contGroup(contGroup) {

}

void Climber::teleopOnUpdate(double dt) {

	if (_contGroup.Get(ControlMap::climber, wml::controllers::Controller::ButtonMode::ONRISE)) {
		if (!ControlMap::ClimberToggle) {
			ControlMap::ClimberToggle = true;
		} else {
			ControlMap::ClimberToggle = false;
		}
	}

	if (!(ControlMap::ClimberToggle)) {
		_leftClimber.SetTarget(wml::actuators::BinaryActuatorState::kForward);
		_rightClimber.SetTarget(wml::actuators::BinaryActuatorState::kForward);
	} else {
		_leftClimber.SetTarget(wml::actuators::BinaryActuatorState::kReverse);
		_rightClimber.SetTarget(wml::actuators::BinaryActuatorState::kReverse);
	}
}