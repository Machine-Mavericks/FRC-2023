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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // create PID controller
    pid = new PIDController(0.02, 0.000, 0.00);
    pid.reset();

    // stop drive
    RobotContainer.swervedrive.drive(0, 0, 0, false, false);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // calculate drive speed from PID controller
    
    double driveSpeed;
    double angle = RobotContainer.gyro.getRoll();
    double ff;

    // if we are witin balanced range, then stop robot
    if (angle <=2.00 && angle>=-2.00)
      driveSpeed = 0.0;
    else
      // we are not balanced, use PID to control robot speed
      driveSpeed = pid.calculate(RobotContainer.gyro.getRoll());
    
    // control feedforward 
    ff=-RobotContainer.gyro.getRoll()*0.007;

    // drive robot to balance
    RobotContainer.swervedrive.drive(MathUtil.clamp(driveSpeed+ff, -0.25, 0.25), 0, 0, false, false);

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
