// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import frc.robot.subsystems.Arm;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DelayCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SemiAutoShelfPickup extends SequentialCommandGroup {
  /** Creates a new SemiAutoShelfPickup. */
  public SemiAutoShelfPickup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      // start wtih drive turned off
      new InstantCommand(() -> RobotContainer.swervedrive.drive(0.0, 0.0, 0.0, false, false)),
    
      // enable arm, and lift to pickup position
      new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

      // lift arm
      new InstantCommand(() -> RobotContainer.arm.SetArmPosition(202.9)), // was 210.9 // was 204.9

      // delay until arm gets back
      new DelayCommand(1.8),

      // open gripper
      new InstantCommand(() -> RobotContainer.grabber.setPosition(RobotContainer.grabber.getPosition().Open)),

      // drive to pickup
      new DriveToShelfPickup(),

      // close gripper
      new InstantCommand(() -> RobotContainer.grabber.setPosition(RobotContainer.grabber.getPosition().Close)),

      // delay until arm gets back
      new DelayCommand(0.5),

      // lift arm
      new InstantCommand(() -> RobotContainer.arm.SetArmPosition(220.0))

    );
  }
}
