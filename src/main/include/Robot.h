// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <rev/ColorSensorV3.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <Mappings.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include <frc/Compressor.h>
#include <frc/DriverStation.h>
#include <frc/Relay.h>
#include <frc/Servo.h>
#include <frc/Encoder.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
class Robot : public frc::TimedRobot {
  frc::Joystick DriveStick{0};
  frc::Talon lDrive0{PWMCHANNELDRIVEL0};
  frc::Talon lDrive1{PWMCHANNELDRIVEL1};
  frc::Talon rDrive0{PWMCHANNELDRIVER0};
  frc::Talon rDrive1{PWMCHANNELDRIVER1};
  frc::MotorControllerGroup lDrive{lDrive0, lDrive1};
  frc::MotorControllerGroup rDrive{rDrive0, rDrive1};
  frc::DifferentialDrive drive{lDrive, rDrive};

  //Effector motors
  frc::Talon cpMotor{PWMCHANNELCPMOTOR};
  frc::Talon climb{PWMCHANNELCLIMBM};
  frc::Servo climbSRVO{PWMCHANNELCLIMBSRVO};
  frc::Talon pickupM{PWMCHANNELPICKUP};
  frc::Talon ShootWhlA{PWMCHANNELSHOOTWHLA};
  frc::Talon ShootWhlB{PWMCHANNELSHOOTWHLB};
  frc::Relay ShootPosMotor{RELAYCHANNELSHOOTPOS, ShootPosMotor.kBothDirections};
  frc::Talon BeltZ1A{PWNCHANNELBELTZ1A};
  frc::Talon BeltZ1B{PWMCHANNELBELTZ1B};
  frc::Talon BeltZ2{PWMCHANNELBELTZ2};
  frc::Talon BeltZ3{PWMCHANNELBELTZ3};
  frc::MotorControllerGroup ShootWhl{ShootWhlA, ShootWhlB};
  frc::MotorControllerGroup BeltZ1{BeltZ1A, BeltZ1B};
  //Pneumatics
  frc::Compressor compressor{0,frc::PneumaticsModuleType::CTREPCM};
  frc::DoubleSolenoid colorSensorArm{0, frc::PneumaticsModuleType::CTREPCM, PCMARTICULATOROUT, PCMARTICULATORIN};
  frc::DoubleSolenoid pickupPneumatic{0, frc::PneumaticsModuleType::CTREPCM, PCMPICKUPOUT, PCMPICKUPIN};
  frc::DoubleSolenoid bottomTension{0, frc::PneumaticsModuleType::CTREPCM, PCMBOTTOMTENSIONF, PCMBOTTOMTENSIONR};
  frc::DoubleSolenoid topTension{0, frc::PneumaticsModuleType::CTREPCM, PCMTOPTENSIONF, PCMTOPTENSIONR};
  frc::Encoder ShootPosEncoder{DIOSHOOTPOSA, DIOSHOOTPOSB,false,frc::Encoder::k4X};
	frc::Encoder leftDriveEncoder{DIGCHANNELLEFTDRIVEA,DIGCHANNELLEFTDRIVEB,false,frc::Encoder::k4X};
	frc::Encoder rightDriveEncoder{DIGCHANNELRIGHTDRIVEA,DIGCHANNELRIGHTDRIVEB,false,frc::Encoder::k4X};
  frc::Counter ShootSpeed{DIGCHANNELSHOOTSPEED};
  static constexpr auto i2cPort0 = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor0{i2cPort0};

  nt::NetworkTableInstance inst;
  std::shared_ptr<nt::NetworkTable> controlTable;
  std::shared_ptr<nt::NetworkTable> sensorTable;
  std::vector<double> clrs;
  unsigned long lastnum = 0;
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  frc::DoubleSolenoid::Value ConvertPNM(std::string_view);

 private:

};
