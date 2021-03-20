// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.commands.CutoffSolenoid;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.EncoderDrive;
import frc.robot.commands.ExtendSolenoid;
import frc.robot.commands.GetNavxValues;
import frc.robot.commands.RetractSolenoid;
import frc.robot.commands.Stop;
//import frc.robot.commands.ToggleSolenoid;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Stop stop;
  //private final ToggleSolenoid toggleSolenoid;
  private final ExtendSolenoid extendSolenoid;
  private final RetractSolenoid retractSolenoid;
  //private final CutoffSolenoid cutoffSolenoid;
  private final GetNavxValues getNavxValues;

  public static Joystick flightstick;
  public static JoystickButton flightstickTrigger;
  public static JoystickButton flightstickButton2;
  public static JoystickButton flightstickButton3;
  public static JoystickButton flightstickButton11;
  
  public static Joystick flightstick2;
  public static JoystickButton flightstick2Trigger;
  public static JoystickButton flightstick2Button2;

  

  public RobotContainer() {

    //Drive Setup
    driveTrain = new DriveTrain();
    driveWithJoystick = new DriveWithJoystick(driveTrain);
    driveWithJoystick.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoystick);

    //Auto command setup
    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    encoderDrive = new EncoderDrive(60, 0.5, driveTrain);
    encoderDrive.addRequirements(driveTrain);
    
    //Other command setup
    pneumatics = new Pneumatics();
    stop = new Stop(driveTrain);
    //toggleSolenoid = new ToggleSolenoid(pneumatics);
    //cutoffSolenoid = new CutoffSolenoid(pneumatics);
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
    // An ExampleCommand will run in autonomous
    return encoderDrive;
  }
}
