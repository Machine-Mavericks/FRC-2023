// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GamePieceTargeting;
import frc.robot.subsystems.GamePieceTargeting.GamePieceData;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class ConePickupCommand extends CommandBase {
  
  // x, y, rotation PID controllers to get us to the intended destination
  private PIDController m_xController; 
  private PIDController m_yController;
  private PIDController m_rotController;
  
  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.01;
  private final double m_angletolerance = 0.05;

  // max speed, rotational speed
  private double m_maxspeed;
  private double m_maxrotspeed;

  // max timeout in command
  private double m_timeout;
  private Timer m_Timer;

  // target position
  private Pose2d m_targetpose;

  // cone targeting

  private boolean m_canceled = false;
  

  public static double AngleDifference(double angle1, double angle2)
  {
    double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
  }
  
  /** Creates a new PrecisionDriveToTargetRelative. */
  public ConePickupCommand()// double MaxSpeed,
                                  //double MaxRotateSpeed,
                                  //double timeout)
    {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);
    
    // set up PIDs
    m_xController = new PIDController(2.5, 0.002, 0.12);
    m_yController = new PIDController(2.5, 0.002, 0.12);
    m_rotController = new PIDController(0.05, 0.001, 0.0000);

    // record maximum speeds to use
    m_maxspeed = 0.5; //MaxSpeed;
    m_maxrotspeed = 1.0; //MaxRotateSpeed;

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
    }

    // reset and start in this command
    m_Timer.reset();
    m_Timer.start();

    

    
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (getTarget() != null){
      updateTargetPose();
    }
    
    if (!m_canceled){
      manageSwerve();
      System.out.println("Swerving");
    }
    
  }

  private void updateTargetPose(){
    GamePieceData data = RobotContainer.gamepiecetargeting.getTargetPose(); // Robot relative
    Pose2d odometryPose = RobotContainer.swerveodometry.getPose2d(); // Field relative
    
    Rotation2d targetAngle;
    if (data.m_Y == 0){
      targetAngle = new Rotation2d(0);
    }else{
      targetAngle = new Rotation2d(Math.atan(data.m_X / data.m_Y));
    }


    

    //System.out.println("TargetAngle: " + targetAngle.getDegrees());

    
     
    

    m_targetpose = new Pose2d(odometryPose.getX(),  odometryPose.getY(), targetAngle.rotateBy(odometryPose.getRotation()));
    //System.out.println("TargetAngle In Pose: " + m_targetpose.getRotation().getDegrees());
    //System.out.println("Current Angle" + odometryPose.getRotation().getDegrees());


  }

  private GamePieceData getTarget() {
    if (RobotContainer.gamepiecetargeting.isTarget())
    {
      return RobotContainer.gamepiecetargeting.getTargetPose();
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
    double xSpeed = m_xController.calculate(m_targetpose.getX() - CurrentPosition.getX() );
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
    Pose2d CurrentPosition = RobotContainer.swerveodometry.getPose2d();

    if (m_canceled){
      return true;
    }

    // we are finished if we are within erorr of target or command had timeed out
    return (((Math.abs(m_targetpose.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
          (Math.abs(m_targetpose.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
          (Math.abs(m_targetpose.getRotation().getDegrees() - CurrentPosition.getRotation().getDegrees()) < m_angletolerance)) ||
          (m_Timer.hasElapsed(m_timeout)));

  }
}
