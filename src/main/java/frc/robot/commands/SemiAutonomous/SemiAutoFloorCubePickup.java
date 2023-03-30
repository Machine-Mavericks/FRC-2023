// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;


public class SemiAutoFloorCubePickup extends SequentialCommandGroup {
  /** Creates a new SemiAutoFloorCubePickup. */
  public SemiAutoFloorCubePickup() {
    // Add your commands in the addCommands()
    addCommands(

    // tilt camera angle
    new InstantCommand(() -> RobotContainer.cameratilt.setPosition(RobotContainer.cameratilt.TILT_FLOORPICKUP_POS))

    // add commands here

    );
  }
}
