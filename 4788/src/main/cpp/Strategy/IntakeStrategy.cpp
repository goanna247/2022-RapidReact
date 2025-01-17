#include "Strategy/IntakeStrategy.h"
#include <iostream>

IntakeStrategy::IntakeStrategy(std::string name, Intake &intake, Shooter &shooter, Controllers &contGroup) : Strategy(name), _intake(intake), _shooter(shooter), _contGroup(contGroup) {
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
  Requires(&intake);
}

void IntakeStrategy::OnUpdate(double dt) {
  if (_contGroup.Get(ControlMap::indexManualToggleButton, wml::controllers::XboxController::ONRISE)) {
    _intake.setMagState(MagStates::kManual);
  }

  if (_contGroup.Get(ControlMap::indexManualStop, wml::controllers::XboxController::ONRISE)) {
    _intake.setMagState(MagStates::kEmpty);
  }

  if (_contGroup.Get(ControlMap::intakeActuation, wml::controllers::XboxController::ONRISE)) {
    if (_intakeToggle) {
      _intakeToggle = false;
    } else {
      _intakeToggle = true;
    }
  }

  if (_intakeToggle) {
    _intake.setIntakeState(IntakeStates::kDeployed);
  } else {
    _intake.setIntakeState(IntakeStates::kStowed);
  }

  double intakeVoltage = fabs(_contGroup.Get(ControlMap::intake)) > ControlMap::triggerDeadzone ? _contGroup.Get(ControlMap::intake) : 0;
  _intake.setIntake(intakeVoltage);

  double indexVoltage = fabs(_contGroup.Get(ControlMap::testingIndex)) > ControlMap::triggerDeadzone ? _contGroup.Get(ControlMap::testingIndex) : 0;
  _intake.setIndex(indexVoltage);

  if (_contGroup.Get(ControlMap::fire)) {
    if (_shooter.isDone() || _shooter.getShooterState() == ShooterState::kManual) {
      _intake.fireBall();
    }
  } 
}

IntakeAutoStrategy::IntakeAutoStrategy(std::string name, Intake &intake, bool twoBall, bool passive) : Strategy(name), _intake(intake), _twoBall(twoBall), _passive(passive) {
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
  SetPassive(passive);
  Requires(&intake);
}

void IntakeAutoStrategy::OnUpdate(double dt) {
  _intake.setIntakeState(IntakeStates::kDeployed);
  _intake.setIntake(1); //take in target number of balls, in 
  if (!_passive) {
    if (_twoBall && _intake._magState == MagStates::kTwo || !_twoBall && _intake._magState == MagStates::kOne) {
      SetDone();
    }
  }
}


// IntakeAutoPassiveStrategy::IntakeAutoPassiveStrategy(std::string name, Intake &intake) : Strategy(name), _intake(intake) {
//   SetCanBeInterrupted(true);
//   SetCanBeReused(true);
//   Requires(&intake);
//   SetPassive(true);
// }

// void IntakeAutoPassiveStrategy::OnUpdate(double dt) {
//   _intake.setIntakeState(IntakeStates::kDeployed);
//   if (_intake._magState == MagStates::kTwo) {
//     _intake.setIntake(1);
//   }
// }