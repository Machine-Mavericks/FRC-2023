// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.subsystems.Arm;

public class SemiAutoCubeDropOffMed extends SequentialCommandGroup {
  /** Creates a new SemiAutoConeDropOffMed. */
  public SemiAutoCubeDropOffMed() {
    
    // Add your commands in the addCommands() call
    addCommands(

    // move arm back to drop off cone
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(Arm.CUBE_MID_DEG)),

    // change camera pipeline
    //new InstantCommand(() -> RobotContainer.limelight_med.setPipeline(2)),

    // delay until camera pipeline changed
    new DelayCommand(0.05),

    // move to drop off cube
    new DriveToCubeDropOff(3),
    
    // delay to stabilize robot before opening gripper
    new DelayCommand(0.15),

    // open gripper - NEED TO CORRECT as open/close is backwards! - TBD
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to close
    new DelayCommand(0.40),

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG))
    
    );
  }
}
