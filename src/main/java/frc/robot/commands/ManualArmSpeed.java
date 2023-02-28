// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.OI;


public class ManualArmSpeed extends CommandBase {
  /** Creates a new ManualArmSpeed. */
  public ManualArmSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get joystick drive inputs - use methods in OI module to get inputs
    // note: intended to use -ve of joystick input
    double InputSpeed = -OI.getArmSpeed()*0.25;
        
    // implement dead-zoning of joystick inputs
    InputSpeed = Math.abs(InputSpeed) > 0.05 ? InputSpeed : 0;

    //Scale the input speed to deg/second with the controller sending a max of 180 deg/second full scale
    InputSpeed = InputSpeed * 110.0;
    //RobotContainer.arm.ArmSpeed(InputSpeed);
    RobotContainer.arm.ArmSpeed_PosCtrl(InputSpeed);

    //Just teting
    //System.out.println((System.currentTimeMillis()) + " ManualArmSpeed set to: " + InputSpeed);   
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
