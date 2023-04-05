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
    
    // initialize states
    state1 = false;
    state2 = false;
    up = false;
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // calculate drive speed from PID controller
    double angle;
    double ff;

    // get robot horizontal angle from gyro
    angle = RobotContainer.gyro2.getPitch();
    
    // balancing split into two states
    // state1 - true when robot is on angle > 10deg
    // state 2 - true when balancer has tipped
    if (!state1)
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

    // gaines are modified depending on which state robot is in
    // to do balance, very low gain is required for stable control operation
    if (!state2)
      ff = RobotContainer.gyro2.getPitch()*0.04;  // was 0.03 - Mar 1/2023 working value!  
    else
      ff=RobotContainer.gyro2.getPitch()*0.011;  // was 0.014

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
