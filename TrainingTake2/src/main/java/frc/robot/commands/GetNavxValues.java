// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GetNavxValues extends CommandBase {

  private final DriveTrain driveTrain;

  public GetNavxValues(DriveTrain dt) {
    
    driveTrain = dt;

    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Roll");
    System.out.println(driveTrain.getNavxRoll());
    System.out.println("Pitch");
    System.out.println(driveTrain.getNavxPitch());
    System.out.println("Yaw");
    System.out.println(driveTrain.getNavxYaw());


  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished() {
    return true;
  }
}
