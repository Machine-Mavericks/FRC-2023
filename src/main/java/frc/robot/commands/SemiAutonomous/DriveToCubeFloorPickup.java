// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class DriveToCubeFloorPickup extends CommandBase {
  
  // y PID controllers to get us to the intended destination
  private PIDController m_yController; 
  //private PIDController m_omegaController;

  // maximum drive speed to use during command (m/s)
  private double m_maxspeed = 0.8;

  // camera target angle - low-pass filtered
  private double m_targetangle_filtered;

  // target longitudinal speed
  private double m_targetxSpeed;

  // current speed to move forward at
  private double xSpeed;

  // function to prevent driving too far in autonomous
  private boolean m_DelayEnable;
  private double m_Delay;
  private Timer m_Timer;

  /** Creates a new DriveToCubeFloorPickup. */
  public DriveToCubeFloorPickup(boolean DelayEnable, double delay) {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);

    m_DelayEnable = DelayEnable;
    m_Delay = delay;

    m_Timer = new Timer();
  
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
    m_targetangle_filtered = 0.0;

    // reset target x speed
    m_targetxSpeed = -1.0;

    // restart timer
    m_Timer.reset();
    m_Timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // set forward speed
    xSpeed = m_targetxSpeed;

    // low pass filter camera target
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
    
    // finished when sensor distance is <= target from shuffleboard
    return (RobotContainer.grabber.GetSensorDistance() >= RobotContainer.grabber.m_Volts.getDouble(2.25) ||
    (m_DelayEnable && m_Timer.hasElapsed(m_Delay) ));
  }
}
