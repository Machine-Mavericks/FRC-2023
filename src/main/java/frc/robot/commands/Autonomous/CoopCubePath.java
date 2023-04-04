// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.DrivetoRelativePose;
import frc.robot.commands.SemiAutonomous.AutoBalance;

// Auto routine:
// 1_ Drop cube in top co-op place
// 2) Drive onto balancer
// 3) Balance on charger
//
// Initial conditions:
// Robot squared-up against wood, arm preloaded wtih cube
// 

public class CoopCubePath extends SequentialCommandGroup {
  /** Creates a new CoopCubePath. */
  public CoopCubePath() {
    // Add your commands in the addCommands() 
    addCommands(

    // enable arm, and lift to stow position
    new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),
    
    // move arm back to drop off cube
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

    // delay until arm gets back
    new DelayCommand(1.5),
    
    // place cube
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to close
    new DelayCommand(0.7),

    // move arm to 'forward position' but inside robot bumper)
    // move to 135deg
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(135.0)),

    // delay for arm to get to stow
    new DelayCommand(1.5),

    // ensure arm is stowed before it is allow to begin moving over charge station
    new SafetyCheckStowPosition(),

    // drive straight ahead over charge station
    new DrivetoRelativePose(new Pose2d(2.5, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),
    
    // balance
    new AutoBalance()
    
    );
    

    
  }
}
