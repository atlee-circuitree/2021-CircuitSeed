// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//CountsPerInch = (CountsPerPulse * GearReduction) / (WheelDiameter * PI)

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class EncoderDrive extends CommandBase {
  
  double target = 0;
  double speed;
  DriveTrain driveTrain;
  Boolean finished = false;


  public EncoderDrive(double inches, double driveSpeed, DriveTrain dt) {
    
    target = inches * Constants.encoderCountsPerMeter;
    speed = driveSpeed;
    driveTrain = dt;
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();

    System.out.println("Target");
    System.out.println(target);
    
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(target < 0 && driveTrain.getLeftEncoder() > target){

      driveTrain.driveForward(-0.5);

    }
    else if(target > 0 && driveTrain.getLeftEncoder() < target){

      driveTrain.driveForward(0.5);
      
    }
    else{
      driveTrain.stop();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
