// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;


public class DriveSidewaysToConeDropOff extends CommandBase {
  
  // y PID controllers to get us to the intended destination
  private PIDController m_yController; 
  
  // maximum speed
  private double m_maxspeed;

  // target horizontal angle - low pass filtered
  private double m_targethorangle_filtered;

  // pipeline to use
  private int m_camerapipeline;

  /** Creates a new DriveToConeDropOff. */
  public DriveSidewaysToConeDropOff(int pipeline) {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);

    // save copy of the provided camera pipeline to use
    m_camerapipeline = pipeline;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set up PIDs
    m_yController = new PIDController(0.05, 0.00, 0.0);

    // change pipeline of camera
    RobotContainer.limelight_med.setPipeline(m_camerapipeline);

    // set maximum speed used during this command
    m_maxspeed = 0.6; 

    // initialize filtered horizontal target
    if (RobotContainer.limelight_med.isTargetPresent())
      m_targethorangle_filtered = RobotContainer.limelight_med.getHorizontalTargetOffsetAngle();
    else
      m_targethorangle_filtered = 0.0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // forward speed
     double xSpeed = 0.0;

     // assume sideways speed is 0 unless target is detected in camera
     double ySpeed =0.0;
     if (RobotContainer.limelight_med.isTargetPresent())
     {
        m_targethorangle_filtered =  0.85*m_targethorangle_filtered + 0.15*RobotContainer.limelight_med.getHorizontalTargetOffsetAngle();
     }
     else
     {
        m_targethorangle_filtered =  0.85*m_targethorangle_filtered;
     }
 
     ySpeed = m_yController.calculate(m_targethorangle_filtered);

     // limit x and y speeds
       if (xSpeed > m_maxspeed)
       xSpeed = m_maxspeed;
     if (xSpeed < -m_maxspeed)
       xSpeed = -m_maxspeed;
     if (ySpeed > m_maxspeed)
       ySpeed = m_maxspeed; 
     if (ySpeed < -m_maxspeed)
       ySpeed = -m_maxspeed;  
       
     // drive robot according to x,y,rot PID controller speeds
     RobotContainer.swervedrive.drive(xSpeed, ySpeed, 0.0, false, false); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // turn off drive
    //RobotContainer.swervedrive.drive(0.0, 0.0, 0.0, false, false);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finished when <2deg in front of target
    return Math.abs(m_targethorangle_filtered) < 5.0;
  }
}
