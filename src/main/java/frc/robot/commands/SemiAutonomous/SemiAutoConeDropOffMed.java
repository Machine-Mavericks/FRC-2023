// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SemiAutoConeDropOffMed extends SequentialCommandGroup {
  /** Creates a new SemiAutoConeDropOffMed. */
  public SemiAutoConeDropOffMed() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    
    // move arm back to drop off cone
    //new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.MID_DEG)),
   new InstantCommand(() -> RobotContainer.arm.SetArmPosition(Arm.MID_DEG)), // was 208.0

    // change camera pipeline
    new InstantCommand(() -> RobotContainer.limelight_med.setPipeline(1)),

    // delay until camera pipeline changed
    new DelayCommand(0.05),

    // move to drop off cone
    new DriveSidewaysToConeDropOff(1),
    new DriveToConeDropOff(1),

    // delay until arm gets back
    new DelayCommand(1.0),
    
    // open gripper
    new InstantCommand(() -> RobotContainer.grabber.setOpen()),

    // delay for gripper to open
    new DelayCommand(0.25),
    
    // close gripper
    new InstantCommand(() -> RobotContainer.grabber.setClose()),
    
    // delay for gripper to close
    new DelayCommand(0.40),

    // move arm to stow position
    new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.STOW_DEG))
    
    );
  }
}
