// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Pathweaver extends CommandBase {
  
  private final DriveTrain driveTrain;
  private Boolean finished = false;

  String trajectoryJSON = "paths/Test1.wpilib.json";
  Trajectory trajectory = new Trajectory();

  RamseteController ramseteController = new RamseteController();

  Timer timer = new Timer();

  public Pathweaver(DriveTrain dt) {
    
    driveTrain = dt;
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      finished = true;
    }

    timer.reset();
    timer.start();

    driveTrain.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Update odometry.
    driveTrain.updateOdometry();

    if (timer.get() < trajectory.getTotalTimeSeconds()) {
      // Get the desired pose from the trajectory.
      var desiredPose = trajectory.sample(timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = ramseteController.calculate(driveTrain.getPose(), desiredPose);

      // Set the linear and angular speeds.
      driveTrain.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    } else {
      driveTrain.drive(0, 0);
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
