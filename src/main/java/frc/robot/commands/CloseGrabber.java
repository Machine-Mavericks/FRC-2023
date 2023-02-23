// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class CloseGrabber extends CommandBase {
  int i = 0;
  /** Creates a new OpenGrabber. */
  public CloseGrabber() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.grabber.setSpeed(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.grabber.setSpeed(-0.05);
    RobotContainer.grabber.setClosed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (i >= 25) {
      return true;
    }
    return false;
  }
}
