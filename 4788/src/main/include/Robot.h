#pragma once

/**
 * Local Files 
 */
#include "RobotMap.h"
#include "Shooter.h"
#include "Drivebase.h"
#include "Intake.h"
#include "Trajectories.h"
#include "Climber.h"
#include "Auto.h"
#include "Vision.h"


#include "Strategy/ShooterStrategy.h"
#include "Strategy/IntakeStrategy.h"
#include "Strategy/ClimberStrategy.h"
#include "Strategy/GetOutStrategy.h"
#include "Strategy/VisionAlignment.h"

#include "frc/DigitalOutput.h"
#include "frc/Relay.h"


class Robot : public frc::TimedRobot, protected wml::StrategyController, protected wml::NTProvider, protected wml::loops::LoopSystem {
public:

  /**
   * Robot boot initilization.
   * Then robot continuous periodic (regardless of mode [teleop/auto/test etc..])
   */
  void RobotInit() override;
  void RobotPeriodic() override;

  /**
   * In it's dissabled mode, simillar to robot init/periodic. However only executes once
   * robot has been disabled at least once.
   */
  void DisabledInit() override;
  void DisabledPeriodic() override;

  /**
   * When the drivestation starts init auto mode,
   * these functions will execute, once for init. And then continuously for periodic
   */
  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  /**
   * Teleop version of auto, used for manual control
   */
  void TeleopInit() override;
  void TeleopPeriodic() override;

  /**
   * Test mode. Runs only in test mode
   */
  void TestInit() override;
  void TestPeriodic() override;

  void Update(double dt) override;

private:
  RobotMap robotMap;
  Trajectories trajectories;
  wml::Drivetrain *drivetrain;
  Shooter *shooter;
  Intake *intake;
  Climber *climber;
  Vision *vision;

  frc::Relay light{0, frc::Relay::Direction::kForwardOnly};

  Auto _auto;

  bool outToggle = false;
  bool climberToggle = false;
  bool isAiming = false;
  bool isDistance = false;
  bool aimToggle = true;
  bool previousAiming = true;

  wml::actuators::DoubleSolenoid lightRelay{ ControlMap::pcModule, wml::actuators::PneumaticsModuleType::kREV, 3, 7, 0.1, "ahhhh"};
};


//put interupt inside the strategy itself