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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  
  //Drivetrain variables
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;
  CANEncoder leftEncoder;
  CANEncoder rightEncoder;
  DifferentialDrive drive;
  
  //NavX variables
  AHRS ahrs; 
  PIDController turnController;
  
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;

  
  
  public DriveTrain() {
    //Drivetrain initialization
    leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    rightMotor = new CANSparkMax(2, MotorType.kBrushless);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    
    leftEncoder = leftMotor.getEncoder(EncoderType.kHallSensor, 4096);
    rightEncoder = rightMotor.getEncoder(EncoderType.kHallSensor, 4096);

    leftEncoder.setPositionConversionFactor(Constants.encoderPPRMod);

    drive = new DifferentialDrive(leftMotor, rightMotor);

    //NavX initialization
    ahrs = new AHRS(SPI.Port.kMXP);

    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(-180.0f, 180.0f);
    
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
  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
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
  public double getNavxAngle(){
    return ahrs.getAngle();
  }

  public double getPIDOutput(){
    return turnController.calculate(ahrs.getAngle());
  }
  public boolean getPIDIsFinished(){
    return turnController.atSetpoint();
  }
  public void setPIDTarget(double setpoint){
    turnController.setSetpoint(setpoint);
  }

}