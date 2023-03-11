// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import java.lang.Math;

public class CenterOnTarget extends CommandBase {
  private double m_xdif;

  /** Creates a new CenterOnTarget. */
  public CenterOnTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swervedrive,RobotContainer.gamepiecetargeting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xdif = RobotContainer.gamepiecetargeting.getTargetHorAngle()+4;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_xdif = RobotContainer.gamepiecetargeting.getTargetHorAngle()+4;
    RobotContainer.swervedrive.drive(0, 0, 0.01*m_xdif, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swervedrive.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_xdif)<1);
  }
}
