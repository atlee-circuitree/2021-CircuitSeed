// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class RetractSolenoid extends CommandBase {
  private final Pneumatics solenoid;

  public RetractSolenoid(Pneumatics ps) {
    
    solenoid = ps;
    addRequirements(solenoid);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    solenoid.retractSolenoid();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
