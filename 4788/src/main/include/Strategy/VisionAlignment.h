#pragma once

#include <iostream>
#include "Vision.h"
#include "controllers/Controller.h"
#include "RobotMap.h"
#include "RobotControl.h"
#include "control/MotorFilters.h"
#include "DriveToDistanceStrategy.h"
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>


class VisionAlignment : public wml::Strategy {
 public:
  VisionAlignment(std::string name, Drivetrain &drivetrain, bool track);

  void OnStart() override;
  void OnUpdate(double dt) override;

 private:
  Drivetrain &_drivetrain;
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");
  DrivetrainAngleStrategy _drivetrainAngleStrategy;
  double _lastYaw = 0;
  double _accSpeed = 0.2;
  bool _track = false;
  photonlib::PhotonCamera photonCamera{"Harry"};
};  // moves robot to align with tape

class VisionSnapStrat : public wml::Strategy {
 public:
  VisionSnapStrat(std::string name);

  void OnUpdate(double dt) override;

 private:
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");
  // bool isInnerCircle = false;
  double fixPitch1 = 20;
  double fixSpeed1 = 0.5;
  double fixPitch2 = -10;
  double fixSpeed2 = 0.8;
};