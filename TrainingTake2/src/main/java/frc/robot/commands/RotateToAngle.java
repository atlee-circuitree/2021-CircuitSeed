// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveTrain;

public class RotateToAngle extends CommandBase {

  private static DriveTrain driveTrain;
  private static double currentRotationRate = 0;
  private static boolean finished = false;
  private static double timeoutSeconds;
  private static double setpoint;
  private static double speed;
  private static int dsOutputDelay = 0;

  private static Timer timer;
  
  public RotateToAngle(double targetAngle,double maxSpeed, double timeout, DriveTrain dt) {
    
    driveTrain = dt;
    timeoutSeconds = timeout;
    setpoint = targetAngle;
    speed = maxSpeed;

    addRequirements(driveTrain);
    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveTrain.setPIDTarget(setpoint);
    System.out.println("Navx Target");
    System.out.println(setpoint);
    System.out.println("Navx Angle");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Setting the range of the motor output values
    if(driveTrain.getPIDOutput() > 0){
      currentRotationRate = MathUtil.clamp(driveTrain.getPIDOutput(), 0.2, speed);
    }
    else{
      currentRotationRate = MathUtil.clamp(driveTrain.getPIDOutput(), -speed, -0.2);
    }
    
    //Print values to screen
    if(dsOutputDelay >= 25){
      dsOutputDelay = 0;
      System.out.println(driveTrain.getNavxAngle());
      System.out.println(currentRotationRate);
    }
    else{
      dsOutputDelay++;
    }

    if(driveTrain.getPIDIsFinished()){
      driveTrain.stop();
      finished = true;
    }
    else if(timer.get() >= timeoutSeconds){
      driveTrain.stop();
      System.out.println("Turn Timeout Occured");
      finished = true;
    }
    else{
      driveTrain.tankDrive(currentRotationRate, -currentRotationRate);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
