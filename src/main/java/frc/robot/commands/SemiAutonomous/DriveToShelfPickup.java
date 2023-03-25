// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;

public class DriveToShelfPickup extends CommandBase {
  
    // y PID controllers to get us to the intended destination
    private PIDController m_yController; 
    private PIDController m_xController; 
    //private PIDController m_omegaController;

    // maximum drive speed to use during command (m/s)
    private double m_maxspeed = 1.4;

    // Distance from the wall the robot should aim to be at the end of this command (in)
    private double m_targetDistance = 20; 
    private double m_acceptableError = 0.5;

    // target distance - low-pass filtered
    //private double m_targetdist_filtered;

    // camera target angle - low-pass filtered
    private double m_targetangle_filtered;

    // current distance (in)
    private double m_distance;

  /** Creates a new DriveToShelfPickup. */
  public DriveToShelfPickup() {
    
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // set up PIDs
    m_yController = new PIDController(0.03, 0.001, 0.0);  // was 0.022

    m_yController = new PIDController(0.03, 0.001, 0.0); 
    //m_omegaController = new PIDController(0.001, 0.0, 0.0);

    // change pipeline of high camera
    // use #1 for left object, use #2 for right object-side
    //if (RobotContainer.targetselector.IsPickupRightSide())
    RobotContainer.limelight_high.setPipeline(0);
    //else
    //  RobotContainer.limelight_high.setPipeline(1);

    // reset filtered value
    m_targetangle_filtered = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get distance sensor reading
    m_distance = RobotContainer.grabber.GetSensorDistanceInches();
    
    
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
    double ySpeed = m_yController.calculate(m_targetangle_filtered);

    // // Only begin moving towards the target if aiming the correct direction 
    // if (Math.abs(RobotContainer.gyro.getYaw()) < 5){
    //   System.out.println("DEBUG:  WITHIN ANGLE RANGE");
    //   // calculate speed with a PID controller to hit the target distance
      
    // }

    double xSpeed = m_yController.calculate(m_distance, m_targetDistance);
    
    
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
    // we are finished once the distance is close to the target distance
    return (Math.abs(m_distance - m_targetDistance) < m_acceptableError);
  }
}
