#pragma once

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

// #include <WMLRev.h>
#include <WMLCtre.h>

#include <iostream>

#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Spark.h>
#include <frc/DoubleSolenoid.h>
#include <frc/GenericHID.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include "strategy/StrategyController.h"
#include "strategy/MPStrategy.h"
#include "NTProvider.h"
#include "WMLCtre.h"
#include "controllers/Controllers.h"
#include "Gearbox.h"
#include "actuators/BinaryServo.h"
#include "actuators/Compressor.h"
#include "actuators/DoubleSolenoid.h"
#include "actuators/VoltageController.h"
#include "sensors/Encoder.h"
#include "sensors/LimitSwitch.h"
#include "sensors/NavX.h"
#include "sensors/PressureSensor.h"
#include "Drivetrain.h"

#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h> 

#include "Toggle.h"

class Robot : public frc::TimedRobot {
 public:
	void RobotInit() override;
	void RobotPeriodic() override;

	void DisabledInit() override;
	void DisabledPeriodic() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

	// controllers
	frc::XboxController *Driver;
	frc::XboxController *CoDriver;

 private:
	//drivebase 
	wml::TalonSrx *_frontLeftMotor;
	wml::TalonSrx *_backLeftMotor;

	wml::TalonSrx *_frontRightMotor;
	wml::TalonSrx *_backRightMotor;

	//intake
	wml::TalonSrx *_intakeMotor;

	//hammer time 
	wml::TalonSrx *_hammerMotor;
};
