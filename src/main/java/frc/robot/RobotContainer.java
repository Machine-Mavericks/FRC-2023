// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.KenPoseEstimator;
import frc.robot.subsystems.TargetSelect;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveOdometry;
import frc.robot.subsystems.SwervePoseEstimator;
import frc.robot.subsystems.Arm;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.PrecisionDriveToPose;
import frc.robot.commands.PrecisionDriveToTarget;
import frc.robot.commands.AutoDriveToTarget;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.subsystems.LEDBlinkin;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Create instances of robot subsystems
  public static final NavX gyro = new NavX();
  //public static final Pigeon gyro2 = new Pigeon();
  public static final Limelight limelight1 = new Limelight("tags", true);
  public static final SwerveDrive swervedrive = new SwerveDrive();
  public static final SwerveOdometry swerveodometry = new SwerveOdometry();
  //public static final SwervePoseEstimator swerveestimator = new SwervePoseEstimator();
  public static final KenPoseEstimator estimator = new KenPoseEstimator();
  public static final TargetSelect targetselector = new TargetSelect();
  //public static final Grabber grabber = new Grabber();
  public static final LEDBlinkin LEDStrip = new LEDBlinkin();
  public static final Arm arm = new Arm();


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
    
    // TODO: Add your button bindings here
    /*OI.PrecisionMoveButton.onTrue(new PrecisionDriveToPose(new Pose2d(1.0, 1.0, new Rotation2d(3.1415/2.0)),
                                                            false,
                                                            1.0, 3.0, 30.0)
    ); */
    
    
    OI.ArmPickupButton.whileTrue(new InstantCommand(()-> arm.SetArmPosition(Arm.ArmPosition.Pickup)));
    OI.ArmStowButton.whileTrue(new InstantCommand(()-> arm.SetArmPosition(Arm.ArmPosition.Stow)));
    OI.ArmDropOffButton.whileTrue(new InstantCommand(()-> arm.SetArmPosition(Arm.ArmPosition.Dropoff)));

    //OI.PrecisionMoveButton.whileTrue(new PrecisionDriveToTarget());
    //OI.PrecisionMoveButton.whileTrue(new InstantCommand(()-> grabber.setAlternatePosition()));
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
