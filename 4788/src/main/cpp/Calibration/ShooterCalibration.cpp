#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include "Calibration/ShooterCalibration.h"
#include "ControlMap.h"
#include "frc/RobotController.h"

ShooterCalibration::ShooterCalibration(std::string name, Shooter &shooter, Intake &intake, photonlib::PhotonCamera &camera,Controllers &contGroup) : wml::Strategy(name), _shooter(shooter), _intake(intake), _camera(camera), _contGroup(contGroup) {
  Requires(&shooter);
  Requires(&intake);
  SetCanBeInterrupted(true);
}

void ShooterCalibration::OnStart() {
  _speed = CAL_MIN_SPEED;
  _values.finalised = false;
}

void ShooterCalibration::OnUpdate(double dt) {
  // std::cout << "SHOOTER CALIBRATION" << std::endl;
  std::string mode_str;
  if (_state == ShooterCalibrationState::kInit) {
    mode_str = "Init";
  } else if (_state == ShooterCalibrationState::kDriverDriveWait) {
    mode_str = "Driver drive wait";
  } else if (_state == ShooterCalibrationState::kSpinUp) {
    mode_str = "Spin up";
  } else if (_state == ShooterCalibrationState::kFire) {
    mode_str = "Fire";
  } else if (_state == ShooterCalibrationState::kDriverConfirm) {
    mode_str = "Driver Confirm";
  } else if (_state == ShooterCalibrationState::kDone) {
    mode_str = "Done";
  }
  nt::NetworkTableInstance::GetDefault().GetTable("shooterCalibration")->GetEntry("State").SetString(mode_str);

  _intake.setIntake(0.8);

  switch (_state) {
  case ShooterCalibrationState::kInit:
    _state = ShooterCalibrationState::kDriverDriveWait;
    break;

  case ShooterCalibrationState::kDriverDriveWait:
    _trial_num = 0;
    _shooter.setManual(0);
    if (_contGroup.Get(ControlMap::DriveWaitOk, wml::controllers::XboxController::ONRISE)) {
      _state = ShooterCalibrationState::kSpinUp;
    } else if (_contGroup.Get(ControlMap::DriveWaitDone, wml::controllers::XboxController::ONRISE)) {
      _state = ShooterCalibrationState::kDone;
      SetDone();
    }
    break;

  case ShooterCalibrationState::kSpinUp:
    _shooter.setPID(_speed, dt);
    if (_shooter.isDone() && _intake.isIdle()) {
      auto target = _camera.GetLatestResult().GetBestTarget();
      _values.speedSet = _speed;
      _values.batteryVoltage = frc::RobotController::GetBatteryVoltage().value();
      _values.visionArea = target.GetArea();
      _values.visionPitch = target.GetPitch();
      _values.visionYaw = target.GetYaw();
      _state = ShooterCalibrationState::kFire;
    }
    break;

  case ShooterCalibrationState::kFire:
    _intake.fireBall();
    std::cout << "Did it hit? 'A' for HIT, 'B' for MISS" << std::endl;
    std::cout.flush(); 
    _state = ShooterCalibrationState::kDriverConfirm;
    break;

  case ShooterCalibrationState::kDriverConfirm:
    if (_contGroup.Get(ControlMap::DriverConfirmHit, wml::controllers::XboxController::ONRISE)) {
      _values.didHit = true;
      _values.finalised = true;
    } else if (_contGroup.Get(ControlMap::DriverConfirmMiss, wml::controllers::XboxController::ONRISE)) {
      _values.didHit = false;
      _values.finalised = true;
    }

    if (_values.finalised) {
      std::cout << _values.visionArea << "," << _values.visionPitch << "," << _values.visionYaw << "," << _values.speedSet
                << "," << _values.batteryVoltage << "," << _values.didHit << std::endl;
      std::cout.flush();
      _trial_num++;

      if (_trial_num >= 3) {
        _speed = _speed + ((CAL_MIN_SPEED + CAL_MAX_SPEED) / 5);
        _trial_num = 0;
      }

      if (_speed <= CAL_MAX_SPEED) {
        _state = ShooterCalibrationState::kSpinUp;
      } else {
        _state = ShooterCalibrationState::kDriverDriveWait;
      }
      _values.finalised = false;
    }
    break;
  }
}

