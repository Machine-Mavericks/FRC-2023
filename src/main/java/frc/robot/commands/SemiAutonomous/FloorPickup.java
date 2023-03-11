// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.SetArmPosition;
import frc.robot.subsystems.Arm;
import java.lang.Math;
import frc.robot.subsystems.LEDBlinkin;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorPickup extends SequentialCommandGroup {
  /** Creates a new FloorPickup. */
  public FloorPickup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> RobotContainer.LEDStrip.setPattern(LEDBlinkin.LED_PATTERN.HEARTBEATRED)),
      new SetArmPosition(Arm.PICKUP_DEG));
     if (RobotContainer.gamepiecetargeting.isTarget()){
      addCommands(
          new InstantCommand(() -> RobotContainer.grabber.setPosition(RobotContainer.grabber.getPosition().Open))
      );
      double xdif = Math.abs(RobotContainer.gamepiecetargeting.getTargetHorAngle()+4);
      while (xdif > 1.0){
        addCommands(
          new InstantCommand(() -> RobotContainer.swervedrive.drive(0.01*xdif, 0, 0, false, false))
        );
      }
      if (RobotContainer.gamepiecetargeting.getGamePieceDistance() > 13.0){
        addCommands(
          new InstantCommand(() -> RobotContainer.swervedrive.drive(0, 0.2, 0, false, false)),
          new DelayCommand(1),
          new InstantCommand(() -> RobotContainer.grabber.setPosition(RobotContainer.grabber.getPosition().Close)),
          new SetArmPosition(Arm.STOW_DEG)
        );
      }
     }
  }
}
