#pragma once

#include "Calibration.h"
#include "Robot.h"
#include "Shooter.h"

enum class ShooterCalibrationState {
  kInit,
  kDriverDriveWait,
  kSpinUp,
  kFire,
  kDriverConfirm,
  kDone
}; 

struct RecordedValues {
  double visionArea;
  double visionPitch;
  double visionYaw;
  double speedSet;
  double batteryVoltage;
  double _actualSpeed;
  bool didHit = false;
  bool finalised = false;
};

constexpr double CAL_MIN_SPEED = 200;
constexpr double CAL_MAX_SPEED = 400;

class ShooterCalibration : public wml::Strategy {
 public:
  ShooterCalibration(std::string name, Shooter &shooter, Intake &intake, photonlib::PhotonCamera &camera,Controllers &contGroup, RobotMap &robotMap);

  void OnStart() override;
  void OnUpdate(double dt) override;

 private:
  RobotMap &_robotMap;
  Controllers &_contGroup;
  Shooter &_shooter;
  Intake &_intake;
  photonlib::PhotonCamera &_camera;
  ShooterCalibrationState _state{ ShooterCalibrationState::kInit };
  RecordedValues _values; 
  unsigned int _hit_num;
  unsigned int _trial_num;
  double _speed;
 
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");

  RisingEdge detect_A;
  RisingEdge detect_X;
  RisingEdge detect_B;
};
