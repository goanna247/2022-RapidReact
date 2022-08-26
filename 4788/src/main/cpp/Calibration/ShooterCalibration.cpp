#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include "Calibration/ShooterCalibration.h"
#include "ControlMap.h"
#include "frc/RobotController.h"

ShooterCalibration::ShooterCalibration(std::string name, Shooter &shooter, Intake &intake, photonlib::PhotonCamera &camera,Controllers &contGroup, RobotMap &robotMap) : wml::Strategy(name), _shooter(shooter), _intake(intake), _camera(camera), _contGroup(contGroup), _robotMap(robotMap), _hit_num(0) {
  Requires(&shooter);
  Requires(&intake);
  SetCanBeInterrupted(true);
  _state = ShooterCalibrationState::kInit;
}

void ShooterCalibration::OnStart() {
  _speed = CAL_MIN_SPEED;
  _values.finalised = false;
}

void ShooterCalibration::OnUpdate(double dt) {
  // std::cout << "SHOOTER CALIBRATION" << std::endl;
  // std::cout << _visionTable->GetEntry("targetYaw").GetDouble(0) << std::endl;
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

  _intake.setIntake(0.5);

  switch (_state) {
  case ShooterCalibrationState::kInit:
    _state = ShooterCalibrationState::kDriverDriveWait;
    break;

  case ShooterCalibrationState::kDriverDriveWait:
    _trial_num = 0;
    _shooter.setManual(0);
    if (detect_A.Get(_robotMap.xbox3.GetRawButton(wml::controllers::XboxController::kA))) {
      std::cout << "move to spin up" << std::endl;
      _state = ShooterCalibrationState::kSpinUp;
    } else if (detect_X.Get(_robotMap.xbox3.GetRawButton(wml::controllers::XboxController::kX))) {
      std::cout << "move to done" << std::endl;
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
      // _values.visionArea = _visionTable->GetEntry("targetArea").GetDouble(0);
      // _values.visionPitch = target.GetPitch();
      _values.visionPitch = nt::NetworkTableInstance::GetDefault().GetTable("visionFilter")->GetEntry("filteredPitch").GetDouble(0); 
      _values._actualSpeed = _shooter.returnSpeed();
      // shooterGearbox.encoder->GetEncoderAngularVelocity();
      
      // _values.visionYaw = _visionTable->GetEntry("targetYaw").GetDouble(0);
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
    {
      bool skip = detect_X.Get(_robotMap.xbox3.GetRawButton(wml::controllers::XboxController::kX));
      if (detect_A.Get(_robotMap.xbox3.GetRawButton(wml::controllers::XboxController::kA))) {
        _values.didHit = true;
        _values.finalised = true;
      } else if (detect_B.Get(_robotMap.xbox3.GetRawButton(wml::controllers::XboxController::kB))) {
        _values.didHit = false;
        _values.finalised = true;
      }

      if (_values.finalised) {
        if (_values.didHit) {
          auto calibrationDetails = nt::NetworkTableInstance::GetDefault().GetTable("shooterCal");
          calibrationDetails->PutNumber("speed", _values.speedSet);
          calibrationDetails->PutNumber("pitch", _values.visionPitch);
          calibrationDetails->PutNumber("voltage", _values.batteryVoltage);
          calibrationDetails->PutNumber("actual_speed", _values._actualSpeed);
          calibrationDetails->PutNumber("hit_num", _hit_num);
          _hit_num++;
          // nt::NetworkTableInstance::GetDefault().GetTable("calibrationDetails")->GetEntry("calibration ")
        }
        // std::cout << _values.speedSet << "," <<_values.visionArea << "," << _values.visionPitch << "," << _values.visionYaw
        //           << "," << _values.batteryVoltage << "," << _values.didHit << ";" << std::endl;
        std::cout << _values.speedSet << "," << _values.visionPitch << "," << _values.batteryVoltage << "," << _values.didHit << ";" << std::endl;
        std::cout.flush();
        _trial_num++;

        if (_trial_num >= 5 || skip) {
          _speed = _speed + ((CAL_MIN_SPEED + CAL_MAX_SPEED) / 30);
          _trial_num = 0;
        }

        if (_speed <= CAL_MAX_SPEED) {
          _state = ShooterCalibrationState::kSpinUp;
        } else {
          _state = ShooterCalibrationState::kDriverDriveWait;
        }
        _values.finalised = false;
      }
    }
    break;
  }
}



//Shuffle board stuff 
// mag sensors 
// mag mode 
// whether intake is up/down 
// pressure value 
// 
// Shooter at speed 
// camera feed 
// Distance can shoot 