// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.DrivetoRelativePose;

// Auto routine:
// 1_ Drop cube in top co-op place
// 2) Drive ahead over line
//
// Initial conditions:
// Robot squared-up against wood, arm preloaded wtih cube

public class RightPath extends SequentialCommandGroup {
  /** Creates a new RightPath. */
  public RightPath() {
    // Add your commands in the addCommands() 
    addCommands(
    
    // enable arm, and lift to stow position
    new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

    // move arm back to drop off cube
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

    // delay until arm gets back
    new DelayCommand(2.0),
    
    // open gripper
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to open
    new DelayCommand(0.50),

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG)),

    // delay for arm to get to stow
    new DelayCommand(1.5),

    // delay according to shuffleboard
    new AutoDelayCommand(),

    // ensure arm is stowed before it is allow to begin moving over charge station
    new SafetyCheckStowPosition(),

    // drive straight ahead over charge station by 6m
    new DrivetoRelativePose(new Pose2d(5.1, 0, new Rotation2d(0.0)),0.4,0.1, 10.0)
    );
    
  }
}
