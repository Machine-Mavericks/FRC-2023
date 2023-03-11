// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.SetArmPosition;
import frc.robot.subsystems.Arm;

public class FloorDriveUp extends CommandBase {

  private double m_time;
  /** Creates a new FloorDriveUp. */
  public FloorDriveUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swervedrive, RobotContainer.gamepiecetargeting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SetArmPosition(Arm.PICKUP_DEG+4);
    m_time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swervedrive.drive(-0.2, 0, 0, false, false);
    if (RobotContainer.gamepiecetargeting.isTarget()){
      if ((RobotContainer.gamepiecetargeting.getTargetVertAngle() < 15.0)&&(RobotContainer.gamepiecetargeting.getTargetVertAngle() > 0.0)){
        m_time += 0.02;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swervedrive.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_time>=0.02);
  }
}
