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
  
    // maximum speed
    private double m_maxspeed;

    // target distance - low pass filtered
    private double m_targetdist_filtered;

  /** Creates a new DriveToShelfPickup. */
  public DriveToShelfPickup() {
    
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.swervedrive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // set up PIDs
    m_yController = new PIDController(0.05, 0.00, 0.0);

    // change pipeline of high camera
    // use #1 for left object, use #2 for right object-side
    //if (RobotContainer.targetselector.IsPickupRightSide())
      RobotContainer.limelight_high.setPipeline(0);
    //else
    //  RobotContainer.limelight_high.setPipeline(2);

   // set maximum speed used during this command
    m_maxspeed = 0.45;   // was 0.25 @0.35V

    m_targetdist_filtered = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // forward speed
    double xSpeed = 0.4;

    // get sensor distance - limit to 40cm
    double dist = RobotContainer.grabber.GetSensorDistance();
    //if (dist > 400.0)
    //  dist = 400.0;

    // assume sideways speed is 0 unless target is detected in camera
    double ySpeed =0.0;
    if (RobotContainer.limelight_high.isTargetPresent())
    {
      ySpeed = m_yController.calculate( RobotContainer.limelight_high.getHorizontalTargetOffsetAngle());
      m_targetdist_filtered = 0.93*m_targetdist_filtered + 0.07*dist;
      //m_targetdist_filtered = dist;
    }
    else
    {
      //m_targetdist_filtered = 0.93*m_targetdist_filtered;
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
    // command finishes when sensor voltage >1.45V (closer than ~20cm from target)
    //return (m_targetdist_filtered >1.45);  
    return (m_targetdist_filtered >RobotContainer.grabber.m_Volts.getDouble(2.22) ); 
  }
}
