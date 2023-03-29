// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import frc.robot.subsystems.Arm;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.DrivetoRelativePose;
import frc.robot.commands.Autonomous.SafetyCheckRaiseArm;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SemiAutoShelfPickup extends SequentialCommandGroup {
  /** Creates a new SemiAutoShelfPickup. */
  public SemiAutoShelfPickup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      // ensure sufficient distance away from shelf before starting
      new SafetyCheckRaiseArm(),

      // start wtih drive turned off
      new InstantCommand(() -> RobotContainer.swervedrive.drive(0.0, 0.0, 0.0, false, false)),
    
      // enable arm, and lift to pickup position
      new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

      // lift arm
      new InstantCommand(() -> RobotContainer.arm.SetArmPosition(213.0)), // was 210.9 // was 204.9

      // delay until arm gets back
      new DelayCommand(1.0),

      // open gripper
      new InstantCommand(() -> RobotContainer.grabber.setOpen()),

      // drive to pickup
      new DriveToShelfPickup(),

      // drive straight back to clear shelf
      new DrivetoRelativePose(new Pose2d(0.7, 0, new Rotation2d(0.0)),0.25,0.1, 30.0),

      // stow arm
      new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG))
      
      // open gripper, change auto pickup so the gripper spins for longer. 3 sec instead of 1. 
      //new InstantCommand(() -> RobotContainer.grabber.setOpen()),

      // delay until arm gets back
      //new DelayCommand(0.5),

      // lift arm
      //new InstantCommand(() -> RobotContainer.arm.SetArmPosition(220.0))

    );
  }
}
