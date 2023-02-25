// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabGamePiece extends SequentialCommandGroup {
  /** Creates a new GrabGamePiece. */
  public GrabGamePiece() {
    // Add your commands in the addCommands() call, e.g.
    Grabber grabber = RobotContainer.grabber;
    Arm arm = RobotContainer.arm;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> arm.SetEnableArm(true)),
      new InstantCommand(() -> grabber.setPosition(Grabber.GrabberPos.Close)),
      new SetArmPosition(Arm.STOW_DEG),
      new InstantCommand(() -> grabber.setAlternatePosition()),
      new SetArmPosition(Arm.PICKUP_DEG),
      new ConePickupCommand(),
      new InstantCommand(() -> grabber.setAlternatePosition()),
      new SetArmPosition(Arm.STOW_DEG)
    );
  }
}
