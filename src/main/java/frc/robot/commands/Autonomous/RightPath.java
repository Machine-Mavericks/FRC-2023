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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightPath extends SequentialCommandGroup {
  /** Creates a new RightPath. */
  public RightPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
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

    // move arm to stow position - 25deg  (aka 'forward position' but inside robot bumper)
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG-25.0)),

    // delay for arm to get to stow
    new DelayCommand(1.5),

    // delay according to shuffleboard
    new AutoDelayCommand(),

    // ensure arm is stowed before it is allow to begin moving over charge station
    new SafetyCheckStowPosition(),

    // drive straight ahead over charge station by 6m
    new DrivetoRelativePose(new Pose2d(5.1, 0, new Rotation2d(0.0)),0.4,0.1, 10.0)
    );
    
    /**
     drive forward
     place cone on 9,2
      if no place game piece command
        move arm to out position for high pole
        drop game piece (ie. open grabber)
        retract arm
     drive backwards
     drive left to position (5,0)
     drive backwards onto charging station
     balance
    */


    
  }
}
