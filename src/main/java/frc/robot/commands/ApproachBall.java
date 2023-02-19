// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.NavX2;
public class ApproachBall extends CommandBase {
  /** Creates a new ApproachBall. */
  
  private SwerveDrive m_SwerveDrive = RobotContainer.swervedrive;
  private NavX2 m_NavX2 = RobotContainer.NavX2;

// get angle to target
double TargetAngle = 0;

   // TODO: set gains
   double kp = 0.0125;
   double ki = 0.0;
   double kd = 0.0; //0.00015;
 
   PIDController pidController = new PIDController(kp, ki, kd);
  
  
  public ApproachBall() {
    // Use addRequirements() here to declare subsystem dependencies.
  
    addRequirements(m_SwerveDrive);
    addRequirements(m_NavX2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double angle = 0.0;
    double speed = 0.0;

// do we have a valid target?
if ((RobotContainer.gamePieceTargeting.IsTarget())){

  
  TargetAngle = RobotContainer.gamePieceTargeting.getTargetHorAngle();
    
  // determine angle correction - uses PI controller
  angle = pidController.calculate(TargetAngle+5.0);
  if (angle > 1.0)
    angle = 1.0;
  if (angle < -1.0)
    angle = -1.0;

  if (m_NavX2.getYaw() >-80)
    speed = 0.2;
  else
    speed = 0.2 + 0.10*(-90 - m_NavX2.getYaw())/10.0;

if (speed < 0.0)
  speed = 0.0;

  }   // end if we have a valid target


    RobotContainer.swervedrive.drive(
      new Translation2d(speed * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
          0 * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
          angle * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.swervedrive.drive(
      new Translation2d(0.0 * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
          0 * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
          0.0* SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angle = m_NavX2.getYaw();
    
    return ((angle >=87 && angle <=93) ||
    (angle <-87 && angle >=-93));

  }
}
