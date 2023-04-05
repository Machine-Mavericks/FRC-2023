// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoWaitArmToBeAtDestination extends CommandBase {
  
  double m_target;

  /** Creates a new AutoWaitArmToBeAtDestination. */
  public AutoWaitArmToBeAtDestination(double targetpos) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_target = targetpos;
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
    
    return Math.abs(RobotContainer.arm.GetArmPosition() - m_target) <= 10.0;
    
  }
}
