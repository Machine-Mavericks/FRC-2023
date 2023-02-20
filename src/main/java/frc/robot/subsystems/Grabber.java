// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Grabber extends SubsystemBase {
  private Boolean isOpen;
  private CANSparkMax grabberMotor;
  /** Creates a new Grabber. */
  public Grabber() {
    grabberMotor = new CANSparkMax(RobotMap.CANID.GRABBER_MOTOR_ID, MotorType.kBrushed);
    grabberMotor.restoreFactoryDefaults();
    grabberMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void stop() {
    grabberMotor.stopMotor();
  } 
  public void setSpeed(double speed) {
    grabberMotor.set(speed);
  }
  public Boolean isOpen() {
    return isOpen;
  }
  public void setClosed() {
    isOpen = false;
  }
  public void setOpen() {
    isOpen = true;
  }
}
