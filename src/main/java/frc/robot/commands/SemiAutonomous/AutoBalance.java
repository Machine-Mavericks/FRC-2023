// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;


public class AutoBalance extends CommandBase {
  
  // PID controlelr for balancing
  PIDController pid;
  
  /** Creates a new AutoBalance. */
  public AutoBalance() {
    
  // this command requires use of swervedrive subsystem
  addRequirements(RobotContainer.swervedrive);

  }

  private boolean state1;
  private boolean state2;
  private boolean up = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // create PID controller
    pid = new PIDController(0.0, 0.000, 0.01);
    pid.reset();

    // stop drive
    RobotContainer.swervedrive.drive(0, 0, 0, false, false);
    
    state1 = false;
    state2 = false;
    up = false;
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // calculate drive speed from PID controller
    
    double driveSpeed;
    //double angle = RobotContainer.gyro.getPitch();
    double angle = RobotContainer.gyro2.getPitch();
    double ff;

    //pid.setP(Math.min(0.5, Math.abs(angle)*0.03));
    
    //driveSpeed = -pid.calculate(angle);
    //RobotContainer.swervedrive.drive(MathUtil.clamp(driveSpeed, -0.25, 0.25), 0, 0, false, false);

    // working??
    /*if (angle >10 )
      RobotContainer.swervedrive.drive(-0.15, 0, 0, false, false);
    else
      RobotContainer.swervedrive.drive(0.0, 0, 0, false, true);
*/

    // if we are witin balanced range, then stop robot
    //if (angle < -12.0)
    //  RobotContainer.swervedrive.drive(0.4, 0, 0, false, false);
   // else if (angle >= -10.0)
   //   RobotContainer.swervedrive.drive(0.0, 0, 0, false, true);
      
    //if (angle <=2.00 && angle>=-2.00)
    //  driveSpeed = 0.0;
    //else
      // we are not balanced, use PID to control robot speed
      //driveSpeed = 0.2;
      //driveSpeed = pid.calculate(angle);
    
    // control feedforward 
    //ff=0;
    
    //angle = RobotContainer.gyro.getPitch();
    angle = RobotContainer.gyro2.getPitch();
    if (!state1 )
    {
      if (angle>10.0)
      { state1 = true; up = true;}

      else if (angle < -10.0)
      { state1 = true; up = false; }
    }

    if (state1 && !state2)
    {
      if (up && angle <-0.0)
        state2 = true;
      else if (!up && angle >0.0)
        state2 = true;

    }

    if (!state2)
      ff = RobotContainer.gyro2.getPitch()*0.04;  // was 0.03 - Mar 1/2023 working value!  
      //ff = RobotContainer.gyro.getPitch()*0.04;  // was 0.03 - Mar 1/2023 working value!
    else
      ff=RobotContainer.gyro2.getPitch()*0.011;  // was 0.014
      //ff=RobotContainer.gyro.getPitch()*0.011;  // was 0.014

    // drive robot to balance
    RobotContainer.swervedrive.drive(MathUtil.clamp(ff, -0.4, 0.4), 0, 0, false, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     RobotContainer.swervedrive.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
