// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class DrivetoFieldPose extends CommandBase {
  
  // target position, speed and rotational speed
  private Pose2d m_target;
  private double m_speed;
  private double m_rotspeed;
  
  // command timeout and counter
  private double m_timeout;
  private Timer m_Timer;

  // x, y, rotation PID controllers to get us to the intended destination
  private PIDController m_xController; 
  private PIDController m_yController;
  private PIDController m_rotController;

  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.01;
  private final double m_angletolerance = 0.5;

  final float DEGtoRAD = (float) (3.1415926 / 180.0);

  public static double AngleDifference(double angle1, double angle2)
  {
    double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
  }
  
  /** Creates a new DrivetoFieldPose. */
  public DrivetoFieldPose(Pose2d target,
                          double speed,
                          double rotationalspeed,
                          double timeout) {
    
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);
    
    // set up PIDs
    m_xController = new PIDController(2.5, 0.002, 0.0);
    m_yController = new PIDController(2.5, 0.002, 0.0);
    m_rotController = new PIDController(0.05, 0.001, 0.0000);
   
    // record target
    m_target = target;

    // record speed limits
    m_speed = speed;
    m_rotspeed = rotationalspeed;

    // create timer, and record timeout limit
    m_Timer = new Timer();
    m_timeout = timeout;                        
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset and start in this command
    m_Timer.reset();
    m_Timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    // get current position estimate from estimator
    Pose2d pose = new Pose2d(); //= RobotContainer.poseestimator.getPose2d();

    // only integrate errors in within 10cm or 5deg of target
    if (Math.abs(m_target.getX() - pose.getX())<0.10)
    m_xController.setI(0.2);
    else
    m_xController.setI(0.0);
    if (Math.abs(m_target.getY() - pose.getY())<0.10)
    m_yController.setI(0.2);
    else
    m_yController.setI(0.0);
    if (Math.abs(AngleDifference(m_target.getRotation().getDegrees(),pose.getRotation().getDegrees()))<5.0)
    m_rotController.setI(0.05);
    else
    m_rotController.setI(0.0);

    // execute PIDs
    double xSpeed = m_xController.calculate(m_target.getX() - pose.getX() );
    double ySpeed = m_yController.calculate( m_target.getY() - pose.getY());
    double rotSpeed = m_rotController.calculate(AngleDifference(m_target.getRotation().getDegrees(),pose.getRotation().getDegrees()));

    // limit speeds to allowable
    if (xSpeed > m_speed)
      xSpeed = m_speed;
    if (xSpeed < -m_speed)
      xSpeed = -m_speed;
    if (ySpeed > m_speed)
      ySpeed = m_speed; 
    if (ySpeed < -m_speed)
      ySpeed = -m_speed;  
    if (rotSpeed >m_rotspeed)
      rotSpeed = m_rotspeed;
    if (rotSpeed < -m_rotspeed)
      rotSpeed = -m_rotspeed;

    // drive robot according to x,y,rot PID controller speeds
    RobotContainer.swervedrive.drive(xSpeed, ySpeed, rotSpeed, true, false); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     // we have finished path. Stop robot
     RobotContainer.swervedrive.drive(0.0, 0.0, 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d CurrentPosition = new Pose2d(); //= RobotContainer.poseestimator.getPose2d();

    // we are finished if we are within erorr of target or command had timeed out
    return (((Math.abs(m_target.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
          (Math.abs(m_target.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
          (Math.abs(m_target.getRotation().getDegrees() - CurrentPosition.getRotation().getDegrees()) < m_angletolerance)) ||
          (m_Timer.hasElapsed(m_timeout)));
  }
}
