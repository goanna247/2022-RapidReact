#pragma once

#include "strategy/Strategy.h"
#include "Drivetrain.h"
#include "Intake.h"
#include "Shooter.h"

class Auto {
 public:
  std::shared_ptr<wml::Strategy> FiveBallTerminal(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> ThreeBallTerminal(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> ThreeBallHanger(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> OneTwoBallAuto(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> TurningTest(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> Vision(wml::Drivetrain &drivetrain);
  std::shared_ptr<wml::Strategy> SnapStratTest();
};