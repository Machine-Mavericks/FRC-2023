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
  @Override
  public void execute() {
    // get joystick drive inputs - use methods in OI module to get inputs
    double dX = OI.getXDriveInput()*1.25;
    double dY = OI.getYDriveInput()*1.25;
    double omega = OI.getRotateDriveInput()*1.25;
    boolean Park = OI.getParkButton();

    // command robot to drive
    // swap x, y, omega as necessary to get robot driving with desired axes
    RobotContainer.swervedrive.drive(dX, dY, omega, true,Park);
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
