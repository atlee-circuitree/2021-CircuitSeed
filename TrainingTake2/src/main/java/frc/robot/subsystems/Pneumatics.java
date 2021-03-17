// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  
  DoubleSolenoid solenoid;

  public Pneumatics() {
    //DoubleSolenoid(PCU, Forward port, Reverse port)
    solenoid = new DoubleSolenoid(0, 5, 4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void extendSolenoid(){
    solenoid.set(Value.kForward);
  }
  public void retractSolenoid(){
    solenoid.set(Value.kReverse);
  }
  public void toggleSolenoid(){
    solenoid.toggle();
  }
  public void cutoffSolenoid(){
    solenoid.set(Value.kOff);
  }
}
