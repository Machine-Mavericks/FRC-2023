// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.NavX2;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveOdometry;
import frc.robot.subsystems.SwervePoseEstimator;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.PrecisionDriveToPose;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ResetGyroCommand;
import edu.wpi.first.math.geometry.Pose2d;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Create instances of robot subsystems
  public static final NavX gyro = new NavX();
  //public static final NavX2 gyro2 = new NavX2();
  public static final Limelight limelight1 = new Limelight("tags");
  public static final SwerveDrive swervedrive = new SwerveDrive();
  public static final SwerveOdometry swerveodometry = new SwerveOdometry();
  public static final SwervePoseEstimator swerveestimator = new SwervePoseEstimator();
  

  /* Constructor */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /** Initialise the container for the robot. Contains subsystems, OI devices, and
   * commands. */
  public static void init() {
    // set swerve drive default command to manual driving mode
    swervedrive.setDefaultCommand(new ManualDriveCommand());


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {
    OI.balanceButton.whileTrue(new BalanceCommand(gyro, swervedrive));
    OI.resetButton.onTrue(new ResetGyroCommand(gyro));
    // TODO: Add your button bindings here
    /*OI.PrecisionMoveButton.onTrue(new PrecisionDriveToPose(new Pose2d(1.0, 1.0, new Rotation2d(3.1415/2.0)),
                                                            false,
                                                            1.0, 3.0, 30.0)
    
    ); */
       
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
