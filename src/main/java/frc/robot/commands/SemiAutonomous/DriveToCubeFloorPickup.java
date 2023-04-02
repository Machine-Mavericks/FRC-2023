// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;

public class DriveToCubeFloorPickup extends CommandBase {
  
  // y PID controllers to get us to the intended destination
  private PIDController m_yController; 
  //private PIDController m_omegaController;

  // maximum drive speed to use during command (m/s)
  private double m_maxspeed = 0.8;

  // target distance - low-pass filtered
  private double m_targetdist_filtered;

  // camera target angle - low-pass filtered
  private double m_targetangle_filtered;

  // target longitudinal speed
  private double m_targetxSpeed;

  // current speed to move forward at
  private double xSpeed;

  private double timer;
  private boolean timer_enabled;

  /** Creates a new DriveToCubeFloorPickup. */
  public DriveToCubeFloorPickup() {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set up PIDs
    m_yController = new PIDController(0.05, 0.001, 0.0);  // was 0.03
    //m_omegaController = new PIDController(0.001, 0.0, 0.0);

    // change pipeline of high camera
    RobotContainer.limelight_high.setPipeline(1);

    // reset filtered values
    m_targetdist_filtered = 0.0;
    m_targetangle_filtered = 0.0;

    // reset target x speed
    m_targetxSpeed = -1.0;

    // reset [initial] forward speed
    //xSpeed = m_targetxSpeed;

    timer = 0.0;
    timer_enabled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // set forward speed
    xSpeed = m_targetxSpeed;

    // if (RobotContainer.grabber.GetUltrasonicDistance() < 50.0)
    // {
    //   xSpeed = (xSpeed-0.5)+ 0.5* (RobotContainer.grabber.GetUltrasonicDistance()-20.0)/30.0;
    //   if (xSpeed < 0.5)
    //     xSpeed = 0.5;
    // }

    // get distance sensor reading (in volts) and low pass filter it
    //double dist = RobotContainer.grabber.GetSensorDistance();
    //m_targetdist_filtered = 0.65*m_targetdist_filtered + 0.35*dist;     // 0.8 and 0.2
    
    // low pass filter camera target
    // note: camera filter corner frequency must be sufficiently low to filter out natural wobble frequency of arm (with camera on it)
    // 83 and 17 working very well.
    if (RobotContainer.limelight_high.isTargetPresent())
    {
      m_targetangle_filtered = 0.81*m_targetangle_filtered + 0.19*RobotContainer.limelight_high.getHorizontalTargetOffsetAngle();
    }
    else
    {
      m_targetangle_filtered = 0.81*m_targetangle_filtered;
    }
    
    // determine lateral speed to get on path - determined by PID controller
    double ySpeed = -m_yController.calculate(m_targetangle_filtered);
    
    // give preference to sideways speed over approach speed
    // note the 1.0x factor can be adjusted to change amount of
    // preference robot gives to sideway over approach speed
    xSpeed = xSpeed + Math.abs(1.5*ySpeed);
    if (xSpeed > 0.0)
      xSpeed = 0.0;

    // limit x and y speeds
    if (xSpeed > m_maxspeed)
      xSpeed = m_maxspeed;
    if (xSpeed < -m_maxspeed)
      xSpeed = -m_maxspeed;
    if (ySpeed > 0.6)
      ySpeed = 0.6; 
    if (ySpeed < -0.6)
      ySpeed = -0.6; 
  
  // drive robot according to x,y,rot PID controller speeds
  RobotContainer.swervedrive.drive(xSpeed, ySpeed, 0.0, false, false);  
  
  if (RobotContainer.limelight_high.isTargetPresent() && RobotContainer.limelight_high.getTargetArea() >= 14.0)
  { timer_enabled = true;}
  
  if (timer_enabled)
   timer+=0.02;

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
    // we are finished when cube size >19
    //return (timer > 0.15);
    return (RobotContainer.grabber.GetSensorDistance() >= RobotContainer.grabber.m_Volts.getDouble(2.25));
  }
}
