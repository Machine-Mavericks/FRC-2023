
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class PrecisionDriveToPose extends CommandBase {
  
  private Pose2d m_target;
  private Pose2d m_Pose;
  private boolean m_relative;

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

  public static double AngleDifference(double angle1, double angle2)
  {
    double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
  }

  
  /** Creates a new PrecisionDriveToPose Command. 
   * Use this to have command to drive back to coordinate provided to it
   * Inputs: target - target position
   * relative - is pose2d relative to current position of robot, or absolute?
   * speed (m/s)- robot speed in m/s
   * rotational speed (rad/s?) - robot rotational speed
   * timeout (s) - time before command is forced to complete */
  public PrecisionDriveToPose(Pose2d target,
                              boolean relative,
                              double speed,
                              double rotationalspeed,
                              double timeout) {
    
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);
    
    
    // set up PIDs
    m_xController = new PIDController(2.5, 0.002, 0.12);
    m_yController = new PIDController(2.5, 0.002, 0.12);
    m_rotController = new PIDController(0.05, 0.001, 0.0000);
   
    // record target
    m_Pose = target;

    // record speed limits
    m_speed = speed;
    m_rotspeed = rotationalspeed;

    // record if target is relative
    m_relative = relative;

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

    // determine our target
    // if target is relative to current position
    if (m_relative)
    {
      // get current probot osition
      Pose2d CurrentPosition = RobotContainer.swerveodometry.getPose2d();
      
      // determine angle through which to turn (deg)
      double AngleDiff = AngleDifference(m_Pose.getRotation().getDegrees(),
                                        CurrentPosition.getRotation().getDegrees());
      
      // determine target position and angle
      m_target = new Pose2d(CurrentPosition.getX() + m_Pose.getX(),
                            CurrentPosition.getY() + m_Pose.getY(),
                            new Rotation2d((3.1415926/180.0)*(CurrentPosition.getRotation().getDegrees() + AngleDiff))
                            );
    }
    else
      // target is absolute - simply record
      m_target = m_Pose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d CurrentPosition = RobotContainer.swerveodometry.getPose2d();
    
    // only integrate errors in within 10cm or 5deg of target
    if (Math.abs(m_target.getX() - CurrentPosition.getX())<0.10)
      m_xController.setI(0.2);
    else
      m_xController.setI(0.0);
    if (Math.abs(m_target.getY() - CurrentPosition.getY())<0.10)
      m_yController.setI(0.2);
    else
      m_yController.setI(0.0);
    if (Math.abs(AngleDifference(m_target.getRotation().getDegrees(),CurrentPosition.getRotation().getDegrees()))<5.0)
      m_rotController.setI(0.05);
    else
      m_rotController.setI(0.0);

    // execute PIDs
    double xSpeed = m_xController.calculate(m_target.getX() - CurrentPosition.getX() );
    double ySpeed = -m_yController.calculate( m_target.getY() - CurrentPosition.getY());
    double rotSpeed = m_rotController.calculate(AngleDifference(m_target.getRotation().getDegrees(),CurrentPosition.getRotation().getDegrees()));
    
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
    Pose2d CurrentPosition = RobotContainer.swerveodometry.getPose2d();

    // we are finished if we are within erorr of target or command had timeed out
    return (((Math.abs(m_target.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
          (Math.abs(m_target.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
          (Math.abs(m_target.getRotation().getDegrees() - CurrentPosition.getRotation().getDegrees()) < m_angletolerance)) ||
          (m_Timer.hasElapsed(m_timeout)));
    
  }

}








  
