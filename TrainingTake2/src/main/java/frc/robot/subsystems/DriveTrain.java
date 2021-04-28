// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//CountsPerInch = (CountsPerPulse * GearReduction) / (WheelDiameter * PI)

//Velocity
//MAX Velocity (Full power)

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
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

  TrajectoryConfig trajectoryConfig;
  Trajectory trajectory;
  
  //NavX variables
  AHRS ahrs; 
  PIDController turnController;
  
  //CHANGED kP 4/23 FROM 2.28 to 0.02
  //Based on characterization data
  static final double kP = 0.02;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;

  //Trajectory variables
  DifferentialDriveOdometry odometry;

  SimpleMotorFeedforward feedForward;

  PIDController leftPIDController;
  PIDController rightPIDController;

  DifferentialDriveKinematics kinematics;

  //DEBUGGING
  int debugging = 0;

  
  public DriveTrain() {
    //Drivetrain initialization
    leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    rightMotor = new CANSparkMax(2, MotorType.kBrushless);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    
    leftEncoder = leftMotor.getEncoder(EncoderType.kHallSensor, 4096);
    rightEncoder = rightMotor.getEncoder(EncoderType.kHallSensor, 4096);

    leftEncoder.setPositionConversionFactor(Constants.encoderPPRMod);
    rightEncoder.setPositionConversionFactor(Constants.encoderPPRMod);

    leftEncoder.setVelocityConversionFactor(Constants.encoderVelocityMod);
    rightEncoder.setVelocityConversionFactor(Constants.encoderVelocityMod);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    drive = new DifferentialDrive(leftMotor, rightMotor);

    //NavX initialization
    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.reset();

    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(-180.0f, 180.0f);

    //Trajectory initializaiton
    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());

    feedForward = new SimpleMotorFeedforward(0.158, 2.8);
    
    leftPIDController = new PIDController(kP, kI, kD);
    rightPIDController = new PIDController(kP, kI, kD);
    
    //Double check trackWidth value (in Meters)
    kinematics = new DifferentialDriveKinematics(0.53975);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(ahrs.getRotation2d(), getEncoderMeters(leftEncoder), getEncoderMeters(rightEncoder));
  }

  //DRIVE Functions
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

  public double getEncoderMeters(CANEncoder encoder){
    return encoder.getPosition() / Constants.encoderCountsPerMeter;
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
  public double getNavxVelocityX(){
    return ahrs.getVelocityX();
  }
  public double getNavxVelocityY(){
    return ahrs.getVelocityY();
  }
  public double getNavxVelocityZ(){
    return ahrs.getVelocityZ();
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

  public void zeroHeading() {
    ahrs.reset();
  }
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }
  public double getTurnRate() {
    return -ahrs.getRate();
  }

  //TRAJECTORY Functions
  
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedForward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedForward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = leftPIDController.calculate(rightEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput = rightPIDController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    leftMotor.setVoltage(leftOutput + leftFeedforward);
    rightMotor.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void updateOdometry() {
    odometry.update(ahrs.getRotation2d(), getEncoderMeters(leftEncoder), getEncoderMeters(rightEncoder));
  }
  

  
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(-rightVolts);

    //DEBUGGING PURPOSES ONLY
    if(debugging >= 50){
      System.out.print("Left Volts: ");
      System.out.println(leftVolts);
      System.out.print("Right Volts: ");
      System.out.println(rightVolts);
      debugging = 0;
    }
    else if(leftVolts == 0 && rightVolts == 0){
      System.out.println("FINAL POSE");
      System.out.println(odometry.getPoseMeters());
    }
    else{
      debugging++;
    }
    drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

}