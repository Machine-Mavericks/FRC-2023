// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.subsystems.Arm;

public class SemiAutoConeDropOffMed extends SequentialCommandGroup {
  /** Creates a new SemiAutoConeDropOffMed. */
  public SemiAutoConeDropOffMed() {
    
    // Add your commands in the addCommands() call
    addCommands(
    
    // move arm back to drop off cone
    //new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.MID_DEG)),
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(Arm.MID_DEG)), // was 208.0

    // change camera pipeline
    new InstantCommand(() -> RobotContainer.limelight_med.setPipeline(1)),

    // delay until camera pipeline changed
    new DelayCommand(0.05),

    // move to drop off cone
    new DriveToConeDropOff(1),
    
    // open gripper - NEED TO CORRECT as open/close is backwards! - TBD
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to open
    new DelayCommand(0.40),

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG))
    
    );
  }
}
