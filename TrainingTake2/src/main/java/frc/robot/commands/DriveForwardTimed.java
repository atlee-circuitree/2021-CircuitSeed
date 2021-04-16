// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardTimed extends CommandBase {
  
  double finalEncoderValue = 0;
  double speed;
  DriveTrain driveTrain;
  double encoderCountsPerMeter = (Constants.encoderPPRMod * 10.71) / (0.1524 * Math.PI);
  Boolean finished = false;
  Timer timer;
  double runtime;


  public DriveForwardTimed(double maxSpeed, double time, DriveTrain dt) {
    
    speed = maxSpeed;
    driveTrain = dt;
    runtime = time;
    addRequirements(driveTrain);
    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();
    while(timer.get() < runtime){
      driveTrain.driveForward(speed);
    }
    finalEncoderValue = driveTrain.getLeftEncoder();
    timer.stop();
    System.out.println("Final Encoder Value");
    System.out.println(finalEncoderValue);
    System.out.println("Meters Traveled");
    finalEncoderValue = finalEncoderValue / encoderCountsPerMeter;
    System.out.println(finalEncoderValue);
    System.out.println("Velocity (Meters/Seconds)");
    System.out.println(finalEncoderValue / runtime);


    driveTrain.stop();
    finished = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
