// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;


public class SemiAutoConeDropOffHigh extends SequentialCommandGroup {
  /** Creates a new SemiAutoConeDropOffHigh. */
  public SemiAutoConeDropOffHigh() {
    
    // Add your commands in the addCommands() call, 
    addCommands(
    
    // tilt camera angle
    new InstantCommand(() -> RobotContainer.cameratilt.setPosition(RobotContainer.cameratilt.TILT_CONEDROPOFF_POS))

    /* // move arm back to drop off cone
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

    // change camera pipeline
    new InstantCommand(() -> RobotContainer.limelight_med.setPipeline(0)),

    // delay until camera pipeline changed
    new DelayCommand(0.05),

    // move to drop off cone
    new DriveToConeDropOff(0),
    
    // open gripper - NEED TO CORRECT as open/close is backwards! - TBD
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to open
    new DelayCommand(0.40),

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG))
    */
    );
  }
}
