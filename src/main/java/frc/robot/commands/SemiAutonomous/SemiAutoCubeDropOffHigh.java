// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;


public class SemiAutoCubeDropOffHigh extends SequentialCommandGroup {
  /** Creates a new SemiAutoConeDropOffHigh. */
  public SemiAutoCubeDropOffHigh() {
    // Add your commands in the addCommands() call
    addCommands(
 
    // move arm back to drop off cube
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

    // change camera pipeline
    new InstantCommand(() -> RobotContainer.limelight_low.setPipeline(0)),

    // delay until camera pipeline changed
    new DelayCommand(0.05),

    // move to drop off cube
    new DriveToCubeDropOff(0),
    
    // open gripper - NEED TO CORRECT as open/close is backwards! - TBD
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to close
    new DelayCommand(0.40),

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG))
    
    );
  }
}
