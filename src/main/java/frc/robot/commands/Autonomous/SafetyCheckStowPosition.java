// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SafetyCheckStowPosition extends CommandBase {
  /** Creates a new SafetyCheckStowPosition. */
  public SafetyCheckStowPosition() {
    
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.arm.GetArmPosition() > 135.0 && RobotContainer.arm.GetArmPosition() < 158.0 &&
    RobotContainer.arm.GetArmPositionCANCoder() > 135.0 && RobotContainer.arm.GetArmPositionCANCoder() <158.0);
  }
}
