// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class NavX extends SubsystemBase {
  
// subsystem shuffleboard controls
private GenericEntry m_gyroPitch;
private GenericEntry m_gyroYaw;
private GenericEntry m_gyroRoll;
private GenericEntry m_xAcceleration;
private GenericEntry m_yAcceleration;
private GenericEntry m_pSlider;
private GenericEntry m_iSlider;
private GenericEntry m_dSlider;
private GenericEntry m_maxBalanceSpeedSlider;
private GenericEntry m_calculatedSpeed;
private double kP = 0.00027;
private double kI = 0.0;
private double kD = 200;
private double maxBalanceSpeed = 5;


// make our gyro object
private AHRS gyro;

/** Creates a new Gyro. */
public NavX() {
  //gyro = new AHRS(Port.kMXP);
  gyro = new AHRS(Port.kMXP);
  gyro.reset();
  gyro.calibrate();
  initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  /** Gets the yaw of the robot
   * @return current yaw value (-180 to 180) */
  public double getYaw() {
    // Flip angle since gyro is mounted upside down
    return -gyro.getYaw(); 
  }

  /** Gets the pitch of the robot
   * @return current pitch value (-180 to 180) */
  public double getPitch() {
    return gyro.getPitch();
  }

  /** Resets yaw to zero */
  public void resetGyro() {
    // reset our Gyro
    gyro.reset();
  }

  /** Accumulated yaw
   * @return accumulated angle in degrees */
  public double continuousYaw() {
    return -gyro.getAngle();
  }

  /** Get Roll
   * @return -180 to 180 degrees */
  public double getRoll() {
    return gyro.getRoll();
  }

  /** X Acceleration
   * @return ratio of gravity */
  public double getXAcceleration() {
    return gyro.getRawAccelX();
  }

  /** Y Acceleration
   * @return ratio of gravity */
  public double getYAcceleration() {
    return gyro.getRawAccelY();
  }

  public double getP() {
    return kP;
  }
  public double getI() {
    return kI;
  }
  public double getD() {
    return kD;
  }
  public double getCalcSpeed() {
    PIDController pid = new PIDController(kP, kI, kD);
    return pid.calculate(getPitch(), 0.0);
  }
  public double getMaxBalanceSpeed() {
    return maxBalanceSpeed;
  }

  /** Gyro Shuffleboard */

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("NavX");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("NavX", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_gyroPitch = l1.add("Pitch (deg)", 0.0).getEntry();
    m_gyroYaw = l1.add("Yaw (deg)", 0.0).getEntry();
    m_gyroRoll = l1.add("Roll (deg)", 0.0).getEntry();
    m_xAcceleration = l1.add("X Acceleration", 0.0).getEntry();
    m_yAcceleration = l1.add("Y Acceleration", 0.0).getEntry();
    m_calculatedSpeed = l1.add("PID Controlled Speed", 0.0).getEntry();
    m_pSlider = Tab.add("P", kP)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    .withPosition(2, 0)
    .withSize(4, 1)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    m_iSlider = Tab.add("I", kI)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    .withPosition(2, 1)
    .withSize(4, 1)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    m_dSlider = Tab.add("D", kD)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    .withPosition(2, 2)
    .withSize(4, 1)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    m_maxBalanceSpeedSlider = Tab.add("Max Speed for Balancing", maxBalanceSpeed)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    .withPosition(2, 3)
    .withSize(4, 1)
    .withProperties(Map.of("min", -5, "max", 5))
    .getEntry();
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_gyroPitch.setDouble(getPitch());
    m_gyroYaw.setDouble(getYaw());
    m_gyroRoll.setDouble(getRoll());
    m_xAcceleration.setDouble(getXAcceleration());
    m_yAcceleration.setDouble(getYAcceleration());
    m_calculatedSpeed.setDouble(getCalcSpeed());
    kP = m_pSlider.getDouble(0.4);
    kI = m_pSlider.getDouble(0.0);
    kD = m_pSlider.getDouble(0.15);
    maxBalanceSpeed = m_maxBalanceSpeedSlider.getDouble(0.2);
  }
}
