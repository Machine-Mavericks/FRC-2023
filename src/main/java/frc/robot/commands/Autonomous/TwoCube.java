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
import frc.robot.commands.SemiAutonomous.SemiAutoCubeDropOffHigh;
import frc.robot.commands.SemiAutonomous.SemiAutoCubeDropOffMed;
import frc.robot.commands.SemiAutonomous.SemiAutoFloorCubePickup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCube extends SequentialCommandGroup {

  Pose2d startingPose;
  /** Creates a new TwoCube. */
  public TwoCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // enable arm, and lift to stow position
    new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

    // move arm back to drop off cube
    new SemiAutoCubeDropOffHigh(),

    // delay for arm to get to stow
    new DelayCommand(1.5),

    // ensure arm is stowed before it is allow to begin moving over charge station
    new SafetyCheckStowPosition(),

    // drive straight
    new DrivetoRelativePose(new Pose2d(4.7, 0, new Rotation2d(0.0)),0.8,0.1, 9.0),

    // pick up cube from floor :)
    new SemiAutoFloorCubePickup(),

    // delay
    new DelayCommand(0.5),

    // drive straight back
    new DrivetoRelativePose(new Pose2d(-5.0, 0, new Rotation2d(0.0)),0.8,0.1, 10.0),
    
    // move arm back to drop off cube
    new SemiAutoCubeDropOffMed(),

    // delay for arm to get to stow
    new DelayCommand(1.5),

    // ensure arm is stowed before it is allow to begin moving over charge station
    new SafetyCheckStowPosition(),

    // drive left to center
    new DrivetoRelativePose(new Pose2d(0,2.5, new Rotation2d(0.0)), 1.0, 0.1, 6.0)

    // drive straight ahead over charge station by 6m
    //new DrivetoRelativePose(new Pose2d(2.5, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),
    
    // balance
    //new AutoBalance()
    );
  }
}
