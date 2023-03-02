// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.OI;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Utils;

// This is the default teleop command - used to drive robot based on joystick inputs
public class ManualDriveCommand extends CommandBase {
  
  // PID controller used to counteract rotational drift due to misalignment of wheels
  private PIDController m_headingPID = new PIDController(0.01, 0, 0);
  // Use Double class so it can be set to null
  private Double m_PIDTarget = null;
  private long m_pidDelay = -1;
  
  /** Creates a new ManualDriveCommand. */
  public ManualDriveCommand() {
    addRequirements(RobotContainer.swervedrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PIDTarget = null;
    m_pidDelay = 10; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  double prevdX = 0.0;
  double prevdY = 0.0;
  @Override
  public void execute() {
    // get robot maximum speed
    double maxSpeed = RobotContainer.swervedrive.getMaxSpeed();
    double maxAccel = RobotContainer.swervedrive.getMaxAccel();

    // check - ensure maxAccel is resonable value to ensure robot can slow down in reasonable time
    if (maxAccel < 6.5)
      maxAccel = 6.5;

    // max rate of change of speed
    double MaxChange = 0.02*maxAccel;

    // get joystick drive inputs - use methods in OI module to get inputs
    
    // get go fast/slow speed factor from joystick trigger


    double inputdX = OI.getXDriveInput();
    double inputdY = OI.getYDriveInput();
    double inputomega = OI.getRotateDriveInput()*0.5;
    boolean Park = OI.DriverButtons.park_Button.getAsBoolean();
    
    // implement dead-zoning of joystick inputs
    inputdX = Math.abs(inputdX) > 0.05 ? inputdX : 0;
    inputdY = Math.abs(inputdY) > 0.05 ? inputdY : 0;
    inputomega = Math.abs(inputomega) > 0.05 ? inputomega : 0;

    // determine desired speeds
    double dX = 0.0, dY=0.0, omega =0.0;
    
    double speedfactor = OI.getGoFast();
    if (speedfactor < 0.5)
     {
      dX = 0.5*inputdX;
      dY = 0.5*inputdY;
      omega = -inputomega*2.0;
     }
    else if (speedfactor >0.5)
    {
      dX = 4.0*inputdX*maxSpeed;
      dY = 4.0*inputdY*maxSpeed;
      omega = -inputomega*4.0*RobotContainer.swervedrive.getMaxRotateSpeed();
    }
    
    // determine desired speeds
    

    
    // --------- Correct robot angle for gyro angle wander --------
    if(omega == 0.0 && !OI.DriverButtons.gyro_reset_Button.getAsBoolean())
    {
      if (m_pidDelay > 0)
        m_pidDelay --;
      else 
      {
        // If the target is unset, set it to current heading
        if(m_PIDTarget == null)
        {
          m_PIDTarget = RobotContainer.gyro.getYaw();
          m_headingPID.reset(); // Clear existing integral term as may accumulate while not in use
        }
      
        // if we are not moving robot, then apply 0 rotation speed to straighten wheels
        if (dX==0.0 && dY==0.0)
          omega = 0.0;
        else         // Compute rotational command from PID controller
          omega = m_headingPID.calculate(Utils.AngleDifference(m_PIDTarget, RobotContainer.gyro.getYaw()));
      }
    } 
    else
    {
      // there is rotational input, or gyro reset was pressed, set target to null so it's properly reset next time
      m_PIDTarget = null;
      m_pidDelay = 10; 
    }

    // --------- end gyro wanter correction functionality */


    // apply acceleration limit to drive
    if ((dX - prevdX) > MaxChange)
      dX = prevdX + MaxChange;
    else if ((dX - prevdX) < (-MaxChange))
      dX = prevdX - MaxChange;
    
    if ((dY - prevdY) > MaxChange)
      dY = prevdY + MaxChange;
    else if ((dY - prevdY) < (-MaxChange))
      dY = prevdY - MaxChange;

    // command robot to drive
    // swap x, y, omega as necessary to get robot driving with desired axes
    RobotContainer.swervedrive.drive(dY, dX, omega, true, Park);

    // record our speeds for use next time
    prevdX = dX;
    prevdY = dY;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
