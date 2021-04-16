// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoystick extends CommandBase {
  
  private final DriveTrain driveTrain;
  private static double dsOutputDelay = 0;

  public DriveWithJoystick(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Print values to screen
    if(dsOutputDelay >= 200){
      dsOutputDelay = 0;
      //Print whatever here
    }
    else{
      dsOutputDelay++;
    }


    //Fast mode
    if(RobotContainer.flightstickTrigger.get()){
      driveTrain.driveWithJoystick(1, RobotContainer.flightstick, RobotContainer.flightstick2);
    }
    //Slow mode
    else if(RobotContainer.flightstick2Trigger.get()){
      driveTrain.driveWithJoystick(0.5, RobotContainer.flightstick, RobotContainer.flightstick2);
    }
    //Normal mode
    else{
      driveTrain.driveWithJoystick(0.75, RobotContainer.flightstick, RobotContainer.flightstick2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
