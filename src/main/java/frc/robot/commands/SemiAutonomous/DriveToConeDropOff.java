// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;


public class DriveToConeDropOff extends CommandBase {
  
  // y PID controllers to get us to the intended destination
  private PIDController m_yController; 
  
  // maximum speed
  private double m_maxspeed;
  
  // target distance - low pass filtered
  private double m_targetarea_filtered;

  // pipeline to use
  private int m_camerapipeline;

  /** Creates a new DriveToConeDropOff. */
  public DriveToConeDropOff(int pipeline) {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);

    m_camerapipeline = pipeline;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set up PIDs
    m_yController = new PIDController(0.12, 0.00, 0.0);

    // change pipeline of high camera
    
    RobotContainer.limelight_med.setPipeline(m_camerapipeline);

    // set maximum speed used during this command
    m_maxspeed = 0.5; 

    m_targetarea_filtered = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // forward speed
     double xSpeed = 0.4;

     if (m_camerapipeline==1)
      xSpeed = 0.2;

     // assume sideways speed is 0 unless target is detected in camera
     double ySpeed =0.0;
     if (RobotContainer.limelight_med.isTargetPresent()==1.0)
     {
       ySpeed = m_yController.calculate( RobotContainer.limelight_med.getHorizontalTargetOffsetAngle());
     }
     else
     {
       ySpeed = m_yController.calculate( 0.0);
     }
 
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

     // update target area
     if (RobotContainer.limelight_med.isTargetPresent()==1.0)
      m_targetarea_filtered = 0.95*m_targetarea_filtered+ 0.05*RobotContainer.limelight_med.getTargetArea();
     else
      m_targetarea_filtered = 0.95*m_targetarea_filtered;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // turn off drive
    RobotContainer.swervedrive.drive(0.0, 0.0, 0.0, false, false);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_camerapipeline==0 && m_targetarea_filtered > 0.20) ||
            (m_camerapipeline==1 && m_targetarea_filtered > 0.32));   // 0.52 is good! keep   
  }
}
