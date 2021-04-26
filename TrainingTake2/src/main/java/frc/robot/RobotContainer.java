// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.EncoderDrive;
import frc.robot.commands.ExtendSolenoid;
import frc.robot.commands.GetNavxValues;
import frc.robot.commands.Pathweaver;
import frc.robot.commands.RetractSolenoid;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.Stop;
//import frc.robot.commands.ToggleSolenoid;
//import frc.robot.commands.CutoffSolenoid;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
// The robot's subsystems and commands are defined here...


  private final DriveTrain driveTrain;
  private final Pneumatics pneumatics;
  private final DriveWithJoystick driveWithJoystick;
  private final DriveForwardTimed driveForwardTimed;
  private final EncoderDrive encoderDrive;
  private final Pathweaver pathweaver;
  private final Stop stop;
  private final ExtendSolenoid extendSolenoid;
  private final RetractSolenoid retractSolenoid;
  private final GetNavxValues getNavxValues;
  private final RotateToAngle rotateToAngle;
  

  public static Joystick flightstick;
  public static JoystickButton flightstickTrigger;
  public static JoystickButton flightstickButton2;
  public static JoystickButton flightstickButton3;
  public static JoystickButton flightstickButton11;
  
  public static Joystick flightstick2;
  public static JoystickButton flightstick2Trigger;
  public static JoystickButton flightstick2Button2;

  private Trajectory trajectory;
  private String trajectoryJSON = "paths/Test2.wpilib.json";

  

  public RobotContainer() {

    //Drive Setup
    driveTrain = new DriveTrain();
    driveWithJoystick = new DriveWithJoystick(driveTrain);
    driveWithJoystick.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoystick);

    //Auto command setup
    driveForwardTimed = new DriveForwardTimed(1, 3, driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    encoderDrive = new EncoderDrive(1, 0.5, driveTrain);
    encoderDrive.addRequirements(driveTrain);

    rotateToAngle = new RotateToAngle(-90, 0.5, 5.0, driveTrain);
    rotateToAngle.addRequirements(driveTrain);

    pathweaver = new Pathweaver(driveTrain);
    pathweaver.addRequirements(driveTrain);
    
    //Other command setup
    pneumatics = new Pneumatics();
    stop = new Stop(driveTrain);
    extendSolenoid = new ExtendSolenoid(pneumatics);
    retractSolenoid = new RetractSolenoid(pneumatics);
    getNavxValues = new GetNavxValues(driveTrain);

    


    
    // Configure the button bindings
    configureButtonBindings();
  }



  private void configureButtonBindings() {

    //Flightstick 1
    flightstick = new Joystick(0);
    flightstickTrigger = new JoystickButton(flightstick, 1);
    flightstickButton2 = new JoystickButton(flightstick, 2);
    flightstickButton3 = new JoystickButton(flightstick, 3);
    flightstickButton11 = new JoystickButton(flightstick, 11);

    //Flightstick 2
    flightstick2 = new Joystick(1);
    flightstick2Trigger = new JoystickButton(flightstick2, 1);
    flightstick2Button2 = new JoystickButton(flightstick2, 2);


    //Button assignments
    flightstick2Button2.whileHeld(stop);
    flightstickButton2.whenPressed(extendSolenoid);
    flightstickButton3.whenPressed(retractSolenoid);
    flightstickButton11.whenPressed(getNavxValues);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //The actual trajectory
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch(IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));

  }
}
