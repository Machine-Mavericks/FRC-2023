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


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoopCubePath extends SequentialCommandGroup {
  /** Creates a new CoopCubePath. */
  public CoopCubePath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    // This section currently being worked on (a work in progress)
    // commented out so robot will not run unexpectedly
  
    // enable arm, and lift to stow position
    new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

    // switch arm to fast control mode
    new InstantCommand(() -> RobotContainer.arm.EnableFast(true)),
    
    // move arm back to drop off cube
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

    // delay until arm gets back
    new DelayCommand(1.5),
    
    // open gripper
    new InstantCommand(() -> RobotContainer.grabber.setOpen()),

    // delay for gripper to open
    new DelayCommand(0.35),
    
    // close gripper
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to close
    new DelayCommand(0.40),

    // move arm to stow position - 25deg  (aka 'forward position' but inside robot bumper)
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG-25.0)),

    // delay for arm to get to stow
    new DelayCommand(1.5),

    // ensure arm is stowed before it is allow to begin moving over charge station
    new SafetyCheckStowPosition(),

    // drive straight ahead over charge station by 6m
    new DrivetoRelativePose(new Pose2d(2.5, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),
    
    // drive straight back onto charge station
    //new DrivetoRelativePose(new Pose2d(-1.7, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),


    // drive straight ahead over charge station by 6m
    //new DrivetoRelativePose(new Pose2d(5, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),
        
    // drive straight back onto charge station
    //new DrivetoRelativePose(new Pose2d(-1.7, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),//


    // balance
    new AutoBalance()
    


    
    );
    
    
    /**
     * place cube on high (5,2)
     * drive backwards over charging station past line
     * drive forwards onto charging station
     * balance/dock
     */

    
  }
}
