// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class ToggleSolenoid extends CommandBase {
  private final Pneumatics solenoid;
  private boolean initialized = false;

  public ToggleSolenoid(Pneumatics ps) {
    
    solenoid = ps;
    addRequirements(solenoid);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(initialized == false){
      solenoid.retractSolenoid();
      initialized = true;
    }
    solenoid.toggleSolenoid();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
