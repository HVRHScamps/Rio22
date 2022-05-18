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
  sensorTable = inst.GetTable("sensor");
}


void Robot::RobotPeriodic() {
  if(controlTable->GetNumber("deadman",0) == lastnum || controlTable->GetBoolean("stop",true)){
    std::cout << "Stopping due to saftey timeout" <<std::endl;
    Abort();
  }
  lastnum = controlTable->GetNumber("deadman",0); // This may be an issue if the robot code is fater than NetworkTables is. Might have to add a counter.
  sensorTable->PutBoolean("Override",!IsAutonomous());
  sensorTable->PutBoolean("Enable",IsEnabled());
  sensorTable->PutNumber("shootEncoder",ShootPosEncoder.Get());
  sensorTable->PutNumber("driveEncoderL",leftDriveEncoder.Get());
  sensorTable->PutNumber("driveEncoderR",rightDriveEncoder.Get());
  sensorTable->PutNumber("shootCounter",ShootSpeed.Get());
  sensorTable->PutNumber("colorProximity",colorSensor0.GetProximity());
  clrs = {colorSensor0.GetColor().red,colorSensor0.GetColor().green,colorSensor0.GetColor().blue};
  sensorTable->PutNumberArray("colorSensor",clrs);
}


void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {
drive.TankDrive(controlTable->GetNumber("driveL",0),controlTable->GetNumber("driveR",0));
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
}
void Robot::TeleopInit() {
  Abort();
}

void Robot::TeleopPeriodic() {
  drive.ArcadeDrive(DriveStick.GetY(),DriveStick.GetX());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

frc::DoubleSolenoid::Value Robot::ConvertPNM(std::string_view key){
  bool val = controlTable->GetBoolean(key,0);
  if(val==1){return frc::DoubleSolenoid::kForward;}
  else if(val==-1){return frc::DoubleSolenoid::kReverse;}
  return frc::DoubleSolenoid::kOff;
}

void Robot::Abort(){
drive.StopMotor();
cpMotor.StopMotor();
climb.StopMotor();
pickupM.StopMotor();
ShootPosMotor.Set(ShootPosMotor.kOff);
ShootWhl.StopMotor();
BeltZ1.StopMotor();
BeltZ2.StopMotor();
BeltZ3.StopMotor();
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
