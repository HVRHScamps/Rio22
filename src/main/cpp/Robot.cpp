// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>
void Robot::RobotInit() {
  colorSensorArm.Set(colorSensorArm.kReverse); //initalize the actuator
  bottomTension.Set(bottomTension.kForward);
  topTension.Set(topTension.kReverse);
  compressor.EnableDigital();
  inst = nt::NetworkTableInstance::GetDefault();
  controlTable = inst.GetTable("control");
  sensorTable = inst.GetTable("sensors");
  leftDriveEncoder.SetDistancePerPulse(1/18.852);
  rightDriveEncoder.SetDistancePerPulse(1/18.852);
}


void Robot::RobotPeriodic() {
  int encReset = (int) controlTable->GetNumber("encRst",0);
  switch (encReset){
    case 1:
      ShootPosEncoder.Reset();
      break;
    case 2:
      leftDriveEncoder.Reset();
      break;
    case 3:
      rightDriveEncoder.Reset();
      break;
    case 4:
      ShootSpeed.Reset();
      break;
  }
  controlTable->PutNumber("encRst",0);
  sensorTable->PutBoolean("Override",!IsAutonomous());
  sensorTable->PutBoolean("Enable",IsEnabled());
  sensorTable->PutNumber("shootEncoder",ShootPosEncoder.Get());
  sensorTable->PutNumber("driveEncoderL",leftDriveEncoder.GetDistance());
  sensorTable->PutNumber("driveEncoderR",rightDriveEncoder.GetDistance());
  sensorTable->PutNumber("shootCounter",ShootSpeed.Get());
  sensorTable->PutNumber("colorProximity",colorSensor0.GetProximity());
  clrs = {colorSensor0.GetColor().red,colorSensor0.GetColor().green,colorSensor0.GetColor().blue};
  sensorTable->PutNumberArray("colorSensor",clrs);
}


void Robot::AutonomousInit() {
rDrive.SetInverted(true);
leftDriveEncoder.SetReverseDirection(true);
}

void Robot::AutonomousPeriodic() {
if(controlTable->GetNumber("deadman",0) == lastnum){
    lnbadcount ++;
  }
else {lnbadcount =0;}
if(lnbadcount > 10){
  std::cout << "Stopping due to saftey timeout" <<std::endl;
  Abort();
}
else {
drive.TankDrive(controlTable->GetNumber("driveL",0),controlTable->GetNumber("driveR",0),false);
ShootWhl.Set(controlTable->GetNumber("shootWhl",0));
double shootPosv = controlTable->GetNumber("shootPos",0); //creates local var to reduse NT calls
if (shootPosv == -1){ShootPosMotor.Set(ShootPosMotor.kReverse);}
else if (shootPosv == 1){ShootPosMotor.Set(ShootPosMotor.kForward);}
else {ShootPosMotor.Set(ShootPosMotor.kOff);}
BeltZ1.Set(controlTable->GetNumber("beltZ1",0));
BeltZ2.Set(controlTable->GetNumber("beltZ2",0));
BeltZ3.Set(controlTable->GetNumber("beltZ3",0));
climb.Set(controlTable->GetNumber("climber",0));
climbSRVO.SetAngle(controlTable->GetNumber("climberServo",0));
pickupM.Set(controlTable->GetNumber("pickupM",0));
cpMotor.Set(controlTable->GetNumber("clrPnlM",0));
colorSensorArm.Set(ConvertPNM("clrPnlPNM"));
pickupPneumatic.Set(ConvertPNM("pickupPNM"));
bottomTension.Set(ConvertPNM("lTensPNM"));
topTension.Set(ConvertPNM("uTensPNM"));
}
lastnum = controlTable->GetNumber("deadman",0);
}
void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  drive.ArcadeDrive(DriveStick.GetY(),DriveStick.GetX());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

frc::DoubleSolenoid::Value Robot::ConvertPNM(std::string_view key){
  float val = controlTable->GetNumber(key,0);
  if(val==1){return frc::DoubleSolenoid::kForward;}
  else if(val==-1){return frc::DoubleSolenoid::kReverse;}
  return frc::DoubleSolenoid::kOff;
}

void Robot::Abort(){
lDrive.Set(0);
rDrive.Set(0);
cpMotor.Set(0);
climb.Set(0);
pickupM.Set(0);
ShootPosMotor.Set(ShootPosMotor.kOff);
ShootWhl.Set(0);
BeltZ1.Set(0);
BeltZ2.Set(0);
BeltZ3.Set(0);
colorSensorArm.Set(colorSensorArm.kReverse);
pickupPneumatic.Set(pickupPneumatic.kReverse);
bottomTension.Set(bottomTension.kForward);
topTension.Set(topTension.kReverse);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
