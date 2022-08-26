#include "Strategy/ShooterStrategy.h"
#include <iostream>
#include "frc/RobotController.h"


ShooterManualStrategy::ShooterManualStrategy(std::string name, Shooter &shooter, Controllers &contGroup) : Strategy(name), _shooter(shooter), _contGroup(contGroup) {
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
  Requires(&shooter);
}

void ShooterManualStrategy::OnUpdate(double dt) {
  double manualFlyWheelPower = fabs(_contGroup.Get(ControlMap::manualFlyWheel)) > fabs(ControlMap::xboxDeadzone) ? _contGroup.Get(ControlMap::manualFlyWheel) : 0;
  // visionTarget = -8.88 * nt::NetworkTableInstance::GetDefault().GetTable("visionFilter")->GetEntry("filteredPitch").GetDouble(0);
  // visionTarget = 446 * _visionTable->GetEntry("targetArea").GetDouble(0) + -3.97 * _visionTable->GetEntry("targetPitch").GetDouble(0);
  
  
  // visionTarget = 13.3367 * _visionTable->GetEntry("targetArea").GetDouble(0) + 0.5548 * _visionTable->GetEntry("targetPitch").GetDouble(0) + 0.2271 * _visionTable->GetEntry("targetYaw").GetDouble(0) + 19.5029 * frc::RobotController::GetBatteryVoltage().value();

  // if ((_visionTable->GetEntry("targetArea").GetDouble(0) != 0 && _visionTable->GetEntry("targetPitch").GetDouble(0) != 0 && _visionTable->GetEntry("targetYaw").GetDouble(0) != 0) && visionTarget > visionSpeedGive) {
  //   visionSpeedGive = visionTarget;
  // }
  // if (_visionTable->GetEntry("targetArea").GetDouble(0) != 0 && _visionTable->GetEntry("targetPitch").GetDouble(0)) {
  //   visionSpeedGive = visionTarget;
  // }
  // std::cout << visionSpeedGive << std::endl;


  if (_contGroup.Get(ControlMap::innerCircleShoot)) {
    _shooter.setPID(ControlMap::Shooter::innerCircleShootValue, dt);
  } else if (_contGroup.Get(ControlMap::outerCircleShoot)) {
    _shooter.setPID(ControlMap::Shooter::outerCircleShootValue, dt);
  } else if (_contGroup.Get(ControlMap::shooterEject)) {
    _shooter.setManual(-ControlMap::Shooter::shooterEjectPower);
  } else if (_contGroup.Get(ControlMap::farShoot)) {
    _shooter.setPID(ControlMap::Shooter::farShootValue, dt);
  } else if (_contGroup.Get(ControlMap::noahShoot)) {
    _shooter.setPID(ControlMap::Shooter::noahShootValue, dt);
  } else if (_contGroup.Get(ControlMap::GetOutBoogalloo)) {
    _shooter.setManual(-1 * 12);
  } 
  // else if (_contGroup.Get(ControlMap::TestShoot)) {
  //   _shooter.setPID(visionSpeedGive, dt);
  // }
  else if (_contGroup.Get(ControlMap::manualFlyBack) > 0.1) {
    manualFlyWheelPower = fabs(_contGroup.Get(ControlMap::manualFlyBack)) > fabs(ControlMap::xboxDeadzone) ? _contGroup.Get(ControlMap::manualFlyBack) : 0;
    _shooter.setManual(-manualFlyWheelPower * 12);
  }
  
  else {
    // auto &motor = _shooter._shooterSystem.shooterGearbox.motor;
    _shooter.setManual(manualFlyWheelPower * 12);
  }
}

ShooterSpinUpStrategy::ShooterSpinUpStrategy(std::string name, Shooter &shooter, double angularVelocity) : Strategy(name), _shooter(shooter), _angularVelocity(angularVelocity) {
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
  SetPassive(true);
  Requires(&shooter);
}


void ShooterSpinUpStrategy::OnUpdate(double dt) {
  _shooter.setPID(_angularVelocity, dt);
}

ShooterShootStrategy::ShooterShootStrategy(std::string name, Shooter &shooter, Intake &intake, double angularVelocity, bool cont) : Strategy(name), _shooter(shooter), _intake(intake), _angularVelocity(angularVelocity), _cont(cont) {
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
  Requires(&shooter);
  // SetTimeout(4);
}

void ShooterShootStrategy::OnUpdate(double dt) {
  _shooter.setPID(_angularVelocity, dt);
  if (!_cont && emptyDB.Get(_intake._magState == MagStates::kEmpty)) {
    SetDone();
  } else if (db.Get(_shooter.isDone() && _intake.isIdle())) {
    _intake.fireBall();
  }
}