// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SetArmPosition;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorPickup extends SequentialCommandGroup {
  /** Creates a new FloorPickup. */
  public FloorPickup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new SetArmPosition(Arm.LOW_DEG));
     if (RobotContainer.gamepiecetargeting.isTarget()){
      addCommands(
          // center on target (depends on camera mount) use gamepiecetargeting.getHorizontalTargetOffset (neccessary?)
          new InstantCommand(() -> RobotContainer.grabber.setPosition(RobotContainer.grabber.getPosition().Open)),
          // drive towards the cone until it reaches a  certain gamepiecetargeting.getGamepieceDistance v
          //new DrivetoPickupTarget(),
          new InstantCommand(() -> RobotContainer.grabber.setPosition(RobotContainer.grabber.getPosition().Close)),
          new SetArmPosition(Arm.STOW_DEG)
        );
     }
  }
}
