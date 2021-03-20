// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//CountsPerInch = (CountsPerPulse * GearReduction) / (WheelDiameter * PI)

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;
  CANEncoder leftEncoder;
  CANEncoder rightEncoder;
  AHRS ahrs; 
    
  DifferentialDrive drive;
  
  public DriveTrain() {
    leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    rightMotor = new CANSparkMax(2, MotorType.kBrushless);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    
    leftEncoder = leftMotor.getEncoder(EncoderType.kHallSensor, 4096);
    rightEncoder = rightMotor.getEncoder(EncoderType.kHallSensor, 4096);

    leftEncoder.setPositionConversionFactor(Constants.encoderPPRMod);

    ahrs = new AHRS(SPI.Port.kMXP);
    
    drive = new DifferentialDrive(leftMotor, rightMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void driveWithJoystick(double speed, Joystick joystick, Joystick joystick2){
    drive.arcadeDrive(-joystick.getY()*speed, joystick2.getX()*speed, true);
  }
  public void driveForward(double speed){
    drive.tankDrive(speed, speed);
  }
  public void stop(){
    drive.stopMotor();
  }
  public double getLeftEncoder(){
    return leftEncoder.getPosition();
  }
  public double getRightEncoder(){
    return rightEncoder.getPosition();
  }
  public double debugging(){
    return leftEncoder.getPositionConversionFactor();
  }

  public void resetEncoders(){
    leftEncoder.getPosition();
    rightEncoder.getPosition();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  //NAVX Functions
  public double getNavxYaw(){
    return ahrs.getYaw();
  }
  public double getNavxRoll(){
    return ahrs.getRoll();
  }
  public double getNavxPitch(){
    return ahrs.getPitch();
  }

}