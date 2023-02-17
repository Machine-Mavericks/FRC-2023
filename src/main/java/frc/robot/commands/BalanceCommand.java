// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveDrive;



public class BalanceCommand extends CommandBase {
  // Use addRequirements() here to declare subsystem dependencies.
  NavX gyro;
  SwerveDrive swervedrive;
  
  /** Supplier for left side output percent */
  DoubleSupplier leftSupplier;
  /** Supplier for right side output percent */
  DoubleSupplier rightSupplier;
  PIDController pid;
  /** Creates a new BalanceCommand. */
  public BalanceCommand(NavX gyro, SwerveDrive swervedrive) {
    this.gyro = gyro;
    this.swervedrive = swervedrive;
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(gyro.getP(), gyro.getI(), gyro.getD());
    swervedrive.drive(0, 0, 0, false);
    pid.reset();
    int i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid = new PIDController(gyro.getP(), gyro.getI(), gyro.getD());
    double driveSpeed = gyro.getPitch() < 7 && gyro.getPitch() > -7 ? 0 : (pid.calculate(gyro.getPitch(), 0.0));
    swervedrive.drive(MathUtil.clamp(driveSpeed, -gyro.getMaxBalanceSpeed(), gyro.getMaxBalanceSpeed()), 0, 0, false);
    System.out.println(driveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swervedrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
