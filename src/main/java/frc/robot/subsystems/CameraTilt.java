// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;


public class CameraTilt extends SubsystemBase {

  // create servo
  Servo m_servo = new Servo(RobotMap.PWMPorts.CAMERA_SERVO_ID);

  private GenericEntry m_cameraTiltPos;
  private GenericEntry m_Test;
  private double m_currentpos;

  // predefined camera tilt angle
  public static final double TILT_FLOORPICKUP_POS = 0.3;
  public static final double TILT_CONEDROPOFF_POS = 0.4;
  public static final double TILT_CUBEDROPOFF_POS = 0.4;


  /** Creates a new CameraTilt. */
  public CameraTilt() {
    initializeShuffleboard();
  
    // set default camera angle
    setPosition(TILT_CONEDROPOFF_POS);
  }

  @Override
  public void periodic() {
    // go ahead and set camera angle
    m_servo.set(m_Test.getDouble(0.5));
    
    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  // sets camera tilt to desired angle Position from 0 to 1
  public void setPosition(double value)
  {
      // limit value to allowable range by camera
      if (value > 0.75)  
        value = 0.75;
      if (value < 0.25)
        value = 0.25;

      m_currentpos = value;
  }

  /** returns current camera pos (from 0 to 1) */
  public double getAngle()
  {
    return m_currentpos;
  }

  
  // -------------------- Shuffleboard Commands --------------------

  /** Initialize subsystem shuffleboard page and controls */
private void initializeShuffleboard() { 
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Camera Tilt");

    ShuffleboardLayout l1 = Tab.getLayout("Camera Tilt", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(2, 4);
    m_cameraTiltPos = l1.add("Pos (0 to 1)", 0.0).getEntry();

    m_Test= Tab.add("Test", 0.5)
    .withPosition(3, 1)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 1.0))
    .getEntry();

  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_cameraTiltPos.setDouble(getAngle());
  }
}
