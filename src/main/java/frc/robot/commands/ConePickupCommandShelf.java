// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FloorGamePieceTargeting;
import frc.robot.Utils;
import frc.robot.Utils.GamePieceData;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class ConePickupCommandShelf extends CommandBase {
  
  // x, y, rotation PID controllers to get us to the intended destination
  private PIDController m_xController; 
  private PIDController m_yController;
  private PIDController m_rotController;
  
  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.01;
  private final double m_angletolerance = 0.5;

  // max speed, rotational speed
  private double m_maxspeed;
  private double m_maxrotspeed;

  // max timeout in command
  private double m_timeout;
  private Timer m_Timer;

  // target position
  private Pose2d m_targetpose;

  // cone targeting
  private double m_idealdistance = 80.0;

  private boolean m_canceled = false;
  private boolean m_notFirstLoop = false;
  private boolean m_aquiredValid = false;
  

// Gyro is mounted backwards compared to last year, if robot is acting erratically, it's probably that

  public static double AngleDifference(double angle1, double angle2)
  {
    double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
  }
  
  /** Creates a new PrecisionDriveToTargetRelative. */
  public ConePickupCommandShelf()// double MaxSpeed,
                                  //double MaxRotateSpeed,
                                  //double timeout)
    {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);
    
    // set up PIDs
    m_xController = new PIDController(2.5, 0.002, 0.12);
    m_yController = new PIDController(2.5, 0.002, 0.12);
    m_rotController = new PIDController(0.08, 0.001, 0.0000);

    // record maximum speeds to use
    m_maxspeed = 1; //MaxSpeed;
    m_maxrotspeed = 1.5; //MaxRotateSpeed;

    // create timer, and record timeout limit
    m_Timer = new Timer();
    m_timeout = 30.0; //timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_canceled = false;

    //m_targetpose = new Pose2d(0, 0, new Rotation2d(0));
    System.out.println("Init");

    if (getTarget() == null){
      m_canceled = true;
    }else{
      updateTargetPose();
      m_notFirstLoop = true;
    }

    // reset and start in this command
    m_Timer.reset();
    m_Timer.start();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_canceled){
      if (getTarget() != null){
        updateTargetPose();
      }
      manageSwerve();

      evaluateIfTargetReached();
    }
  }

  private void evaluateIfTargetReached(){
    Pose2d CurrentPosition = RobotContainer.swerveodometry.getPose2d();

    if ((((Math.abs(m_targetpose.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
    (Math.abs(m_targetpose.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
    (Math.abs(m_targetpose.getRotation().getDegrees() - CurrentPosition.getRotation().getDegrees()) < m_angletolerance)) ||
    (m_Timer.hasElapsed(m_timeout)))){
      m_canceled = true;
    }
  }

  private void updateTargetPose(){
    GamePieceData data = RobotContainer.shelfgamepiecetargeting.getTargetPose(); // Robot relative
    Pose2d odometryPose = RobotContainer.swerveodometry.getPose2d(); // Field relative
    boolean pieceValidThisFrame = RobotContainer.shelfgamepiecetargeting.getGamePieceValid();
    
    // Rotation2d targetAngle;
    // if (data.m_Y == 0){
    //   targetAngle = new Rotation2d(0);
    // }else{
    //   targetAngle = new Rotation2d(-Math.atan(data.m_X / data.m_Y));
    // }
    //double distance = Math.sqrt(Math.pow(dataPose.getX(), 2) + Math.pow(dataPose.getY(), 2)); // Robot relative
    //double distCoefficient; // Use to move point along line
    //if (distance == 0){ // This should REALLY never happen, but would crash the robot
    //  distCoefficient = 0; 
    //} else{
    //  distCoefficient = (distance - m_idealdistance) / distance; 
    //}

    Pose2d dataPose = new Pose2d(data.m_X, data.m_Y, new Rotation2d(Math.toRadians(0))); // Robot relative
  
    // Rotate pose around robot angle
    double X = dataPose.getX() * Math.cos(odometryPose.getRotation().getRadians()) - dataPose.getY() * Math.sin(odometryPose.getRotation().getRadians());
    double Y = dataPose.getY() * Math.cos(odometryPose.getRotation().getRadians()) + dataPose.getX() * Math.sin(odometryPose.getRotation().getRadians());

    // Move back target pose by ideal distance
    Y -= m_idealdistance;

    // Convert cm to meters
    X = X / 100;
    Y = Y / 100;

    // Account for camera on rear of robot
    X = -X;
    Y = -Y;

    // Store last pose
    Pose2d previousPose = m_targetpose;

    if (pieceValidThisFrame){
      m_aquiredValid = true;
    }

    // Messy logic to avoid issues
    if (m_aquiredValid & pieceValidThisFrame){
      // Add rotated pose to current position
      m_targetpose = new Pose2d(odometryPose.getX() - Y,  odometryPose.getY() - X,  m_targetpose.getRotation()); 

      if (m_notFirstLoop){ // Average last two pose estimations
        m_targetpose = new Pose2d((m_targetpose.getX() + previousPose.getX()) / 2, (m_targetpose.getY() + previousPose.getY()) / 2, m_targetpose.getRotation());  
      }
    }else{
      if (m_notFirstLoop){
        if (m_aquiredValid & !pieceValidThisFrame){
          // Do nothing to target pose
        }else{
          m_targetpose = new Pose2d(m_targetpose.getX(), m_targetpose.getY(), m_targetpose.getRotation()); 
        }
      }else{
        m_targetpose = new Pose2d(odometryPose.getX(), odometryPose.getY(), m_targetpose.getRotation()); 
      }
      
    }
  }

  private GamePieceData getTarget() {
    if (RobotContainer.shelfgamepiecetargeting.isTarget())
    {
      return RobotContainer.shelfgamepiecetargeting.getTargetPose();
    }
    return null; // If no target, return null
  }

  private void manageSwerve(){
    // get our current position from swerve odometry
    Pose2d CurrentPosition = RobotContainer.swerveodometry.getPose2d();

    // only integrate errors in within 10cm or 5deg of target
    if (Math.abs(m_targetpose.getX() - CurrentPosition.getX())<0.10)
      m_xController.setI(0.2);
    else
      m_xController.setI(0.0);
    if (Math.abs(m_targetpose.getY() - CurrentPosition.getY())<0.10)
      m_yController.setI(0.2);
    else
      m_yController.setI(0.0);
    if (Math.abs(AngleDifference(m_targetpose.getRotation().getDegrees(),CurrentPosition.getRotation().getDegrees()))<5.0)
      m_rotController.setI(0.05);
    else
      m_rotController.setI(0.0);

    // execute PIDs
    double xSpeed = m_xController.calculate(-m_targetpose.getX() + CurrentPosition.getX() );
    double ySpeed = m_yController.calculate( m_targetpose.getY() - CurrentPosition.getY());
    
    double rotSpeed = m_rotController.calculate(AngleDifference(m_targetpose.getRotation().getDegrees(),CurrentPosition.getRotation().getDegrees()));

    // limit speeds to allowable
    if (xSpeed > m_maxspeed)
    xSpeed = m_maxspeed;
    if (xSpeed < -m_maxspeed)
    xSpeed = -m_maxspeed;
    if (ySpeed > m_maxspeed)
    ySpeed = m_maxspeed; 
    if (ySpeed < -m_maxspeed)
    ySpeed = -m_maxspeed;  
    if (rotSpeed >m_maxrotspeed)
    rotSpeed = m_maxrotspeed;
    if (rotSpeed < -m_maxrotspeed)
    rotSpeed = -m_maxrotspeed;

    // drive robot according to x,y,rot PID controller speeds
    RobotContainer.swervedrive.drive(xSpeed, ySpeed, rotSpeed, true, false);  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // we have finished path. Stop robot
    System.out.println("Ending");
    RobotContainer.swervedrive.drive(0.0, 0.0, 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_canceled;

    // we are finished if we are within erorr of target or command had timeed out
    // return (((Math.abs(m_targetpose.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
    //       (Math.abs(m_targetpose.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
    //       (Math.abs(m_targetpose.getRotation().getDegrees() - CurrentPosition.getRotation().getDegrees()) < m_angletolerance)) ||
    //       (m_Timer.hasElapsed(m_timeout)));

  }
}
