// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;



public class SetArmPosition extends CommandBase {

  private double m_TargetPposition;
  private boolean m_CommandError = false;
  // m_ElapsedTime and m_Timeout are values in ms
  private int m_ElapsedTime = 0;
  private int m_Timeout = 100;

  /** Creates a new SetArmPosition. */
  public SetArmPosition(double SetPosition) {
    m_TargetPposition = SetPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.arm.SetArmPosition(m_TargetPposition)){
      // It's all good, Arm subsystem is happy
    } else {
      // Arm subsystem is not happy
      m_CommandError = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Executes every 20ms, so keep track of the time in ms.
    m_ElapsedTime += 20;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.ArmSpeed_PosCtrl(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    //return (m_CommandError || (Math.abs(RobotContainer.arm.GetArmPosition() - m_TargetPposition) < 1.0) || (m_ElapsedTime >= m_Timeout));
  }

}
