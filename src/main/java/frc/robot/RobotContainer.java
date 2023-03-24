// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.GamePieceTargeting;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveOdometry;
import frc.robot.subsystems.SwervePoseEstimator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.AutoPathSelect;
import frc.robot.subsystems.TargetSelect;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.DrivetoRelativePose;
import frc.robot.commands.ManualArmSpeed;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.Autonomous.CoopCubePath;
import frc.robot.commands.Autonomous.LeftPath;
import frc.robot.commands.Autonomous.RightPath;
import frc.robot.commands.Autonomous.TwoConesPath;
import frc.robot.commands.SemiAutonomous.AutoBalance;
import frc.robot.commands.SemiAutonomous.DriveToConeDropOff;
import frc.robot.commands.SemiAutonomous.DriveToShelfPickup;
import frc.robot.commands.SemiAutonomous.SemiAutoConeDropOffHigh;
import frc.robot.commands.SemiAutonomous.SemiAutoConeDropOffMed;
import frc.robot.commands.SemiAutonomous.SemiAutoShelfPickup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

//import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.LEDBlinkin;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Create robot's shuffboard operator interface
  public static final AutoPathSelect autopathselect = new AutoPathSelect();
  public static final TargetSelect targetselector = new TargetSelect();
  
  // Create instances of robot subsystems
  public static final NavX gyro = new NavX();
  // public static final Pigeon gyro2 = new Pigeon();
  public static final Limelight limelight_high = new Limelight("high");
  public static final Limelight limelight_med = new Limelight("med"); 
  public static final SwerveDrive swervedrive = new SwerveDrive();
  public static final SwerveOdometry swerveodometry = new SwerveOdometry();
  //public static final SwervePoseEstimator poseestimator = new SwervePoseEstimator();
  public static final Arm arm = new Arm();  
  public static final Grabber grabber = new Grabber();
  public static final GamePieceTargeting gamepiecetargeting = new GamePieceTargeting(RobotMap.LimelightOffsets.FLOOR_LIMELIGHT_OFFSET_X, RobotMap.LimelightOffsets.FLOOR_LIMELIGHT_OFFSET_Y);
  public static final LEDBlinkin LEDStrip = new LEDBlinkin();
  

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

    // set default arm command to manual drive mode
    arm.setDefaultCommand(new ManualArmSpeed());

    //LEDStrip.setDefaultCommand(new LEDCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {

    // reset gyro button
    OI.DriverButtons.gyro_reset_Button.onTrue(new InstantCommand(()-> gyro.resetGyro()));
    
    // shelf pickup semi-auto routine
    OI.DriverButtons.shelfpickup_Button.whileTrue(new SemiAutoShelfPickup());
    OI.DriverButtons.DropoffHigh_Button.whileTrue(new SemiAutoConeDropOffHigh());
    OI.DriverButtons.DropoffMed_Button.whileTrue(new SemiAutoConeDropOffMed());
    OI.DriverButtons.auto_balance_Button.whileTrue(new AutoBalance());
    

    // arrm movement buttons
    OI.OperatorButtons.ground_Button.onTrue(new SetArmPosition(Arm.PICKUP_DEG));
    OI.OperatorButtons.mid_Button.onTrue(new SetArmPosition(Arm.MID_DEG));
    OI.OperatorButtons.high_Button.onTrue(new SetArmPosition(Arm.HIGH_DEG));
    OI.OperatorButtons.stow_Button.onTrue(new SetArmPosition(Arm.STOW_DEG));
    OI.OperatorButtons.shelf_pickup_Button.onTrue(new SetArmPosition(Arm.PICKUP_SHELF_DEG));

    // grabber open/close
    OI.OperatorButtons.GrabberButton.whileTrue(new InstantCommand(()-> grabber.setOpen()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
   
    // get autonomous path to run
    int index = RobotContainer.autopathselect.GetSelectedPath();
    
    // return autonomous command to be run
    if (index == 0)
      return new CoopCubePath();
    else if (index == 1)
      return new LeftPath();
    else if (index == 2)
      return new RightPath();
    else
      return new CoopCubePath(); 
  }
}
