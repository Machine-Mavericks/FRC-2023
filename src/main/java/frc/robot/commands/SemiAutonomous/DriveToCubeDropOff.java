// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;


public class DriveToCubeDropOff extends CommandBase {
  
  // y PID controllers to get us to the intended destination
  private PIDController m_yController; 
  private PIDController m_rotController;
  
  // maximum speed
  private double m_maxspeed;
  
  // target distance - low pass filtered
  private double m_targetarea_filtered;

  // target horizontal angle - low pass filtered
  private double m_targethorangle_filtered;

  // pipeline to use
  private int m_camerapipeline;

  public static double AngleDifference(double angle1, double angle2)
  {
    double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
  }


  /** Creates a new DriveToConeDropOff. */
  public DriveToCubeDropOff(int pipeline) {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);

    m_camerapipeline = pipeline;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set up PIDs
    m_yController = new PIDController(0.04, 0.00, 0.0);
    m_rotController = new PIDController(0.05, 0.001, 0.0000);

    // change pipeline of med camera
    RobotContainer.limelight_med.setPipeline(2);

    // set maximum speed used during this command
    m_maxspeed = 0.8; 

    m_targetarea_filtered = 0.0;

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
     double xSpeed = 0.70;

     if (m_camerapipeline==1)
      xSpeed = 0.70;

     // assume sideways speed is 0 unless target is detected in camera
     double ySpeed =0.0;
     if (RobotContainer.limelight_med.isTargetPresent())
     {
        m_targethorangle_filtered =  0.81*m_targethorangle_filtered + 0.19*RobotContainer.limelight_med.getHorizontalTargetOffsetAngle();
     }
     else
     {
        m_targethorangle_filtered =  0.81*m_targethorangle_filtered;
     }
 
     ySpeed = m_yController.calculate(m_targethorangle_filtered);
     
     double rotSpeed = m_rotController.calculate(AngleDifference(0.0, RobotContainer.gyro2.getYaw()));
     // for safety, bound rotational speed to safe maximum
     // robot arm is out so can be easy to lose balance
     // limit to 0.4 rad/s
     if (rotSpeed>0.4)
      rotSpeed=0.4;
     else if (rotSpeed<-0.4)
      rotSpeed=-0.4;

     // give preference to sideways speed over approach speed
     // note the 1.0x factor can be adjusted to change amount of
     // preference robot gives to sideway over approach speed
     xSpeed = xSpeed - Math.abs(2.5*ySpeed);
     if (xSpeed < 0.0)
      xSpeed = 0.0;

     // limit x and y speeds
       if (xSpeed > m_maxspeed)
       xSpeed = m_maxspeed;
     if (xSpeed < -m_maxspeed)
       xSpeed = -m_maxspeed;
     if (ySpeed > 0.5)
       ySpeed = 0.5; 
     if (ySpeed < -0.5)
       ySpeed = -0.5;  
       
     // drive robot according to x,y,rot PID controller speeds
     RobotContainer.swervedrive.drive(xSpeed, ySpeed, rotSpeed, false, false); 

     // update target area
     if (RobotContainer.limelight_med.isTargetPresent())
      m_targetarea_filtered = 0.75*m_targetarea_filtered+ 0.25*RobotContainer.limelight_med.getTargetArea();
     //else
      //m_targetarea_filtered = 0.95*m_targetarea_filtered;
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
    return ((m_camerapipeline==2 && m_targetarea_filtered > 2.60) || // was 3.1
          (m_camerapipeline==3 && m_targetarea_filtered > 2.65));
  }
}
