// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;


public class AutoFloorCubePickup extends SequentialCommandGroup {
  /** Creates a new SemiAutoFloorCubePickup. */
  public AutoFloorCubePickup() {
    // Add your commands in the addCommands()
    addCommands(

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG)),

    // tilt camera angle and set camera pipeline
    new InstantCommand(() -> RobotContainer.cameratilt.setPosition(RobotContainer.cameratilt.TILT_FLOORPICKUP_POS)),
    new InstantCommand(() -> RobotContainer.limelight_high.setPipeline(1)),

    // start grabber intake (note should be renamed to close - TBD)
    new InstantCommand(() -> RobotContainer.grabber.setOpen()),

    // drive to cube
    new DriveToCubeFloorPickup(true, 1.5),

    // set arm position to pickup
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.PICKUP_DEG)),

    // delay until camera pipeline changed
    new DelayCommand(0.75),

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG))

    );
  }
}
