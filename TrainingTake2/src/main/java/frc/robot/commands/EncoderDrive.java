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
  double encoderCountsPerInch = (Constants.encoderPPRMod * 10.71) / (6 * Math.PI);
  Boolean finished = false;
  int PROBLEMCHILD = 0;


  public EncoderDrive(double inches, double driveSpeed, DriveTrain dt) {
    
    target = inches * encoderCountsPerInch;
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

    System.out.println("Position Conversion Factor");
    System.out.println(driveTrain.debugging());
    


    
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(target < 0 && driveTrain.getLeftEncoder() > target){

      //System.out.println(driveTrain.getLeftEncoder());
      driveTrain.driveForward(-0.5);

    }
    else if(target > 0 && driveTrain.getLeftEncoder() < target){
      
      
      //Debugging purposes ONLY
      if(PROBLEMCHILD < 50){
        PROBLEMCHILD++;
      }
      else{
        System.out.println(driveTrain.getLeftEncoder());
        PROBLEMCHILD = 0;
      }


      driveTrain.driveForward(0.5);
      
    }
    else{
      driveTrain.stop();
      finished = true;
    }
    /*
    driveTrain.driveForward(0.5);

    if(PROBLEMCHILD < 50){
      PROBLEMCHILD++;
    }
    else{
      System.out.println(driveTrain.getLeftEncoder());
      PROBLEMCHILD = 0;
    }
    */
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
