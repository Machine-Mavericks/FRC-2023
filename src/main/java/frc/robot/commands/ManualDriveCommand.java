// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.OI;


// This is the default teleop command - used to drive robot based on joystick inputs
public class ManualDriveCommand extends CommandBase {
  /** Creates a new ManualDriveCommand. */
  public ManualDriveCommand() {
    addRequirements(RobotContainer.swervedrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  double prevdX = 0.0;
  double prevdY = 0.0;
  @Override
  public void execute() {
    // get robot maximum speed
    double maxSpeed = RobotContainer.swervedrive.getMaxSpeed();
    double maxAccel = RobotContainer.swervedrive.getMaxAccel();

    // max rate of change of speed
    double MaxChange = 0.02*maxAccel;

    // get joystick drive inputs - use methods in OI module to get inputs
    double inputdX = OI.getXDriveInput();
    double inputdY = OI.getYDriveInput();
    double inputomega = OI.getRotateDriveInput();
    
    // implement dead-zoning of joystick inputs
    inputdX = Math.abs(inputdX) > 0.05 ? inputdX : 0;
    inputdY = Math.abs(inputdY) > 0.05 ? inputdY : 0;
    inputomega = Math.abs(inputomega) > 0.05 ? inputomega : 0;

    // determine desired speeds
    double dX = inputdX*maxSpeed;
    double dY = inputdY*maxSpeed;
    double omega = -inputomega*5.0*RobotContainer.swervedrive.getMaxRotateSpeed();

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
    RobotContainer.swervedrive.drive(-dY, -dX, omega, true);

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
