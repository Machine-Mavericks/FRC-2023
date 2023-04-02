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


// class to tilt high camera (located on arm)

public class CameraTilt extends SubsystemBase {

  // create servo
  Servo m_servo = new Servo(RobotMap.PWMPorts.CAMERA_SERVO_ID);

  private GenericEntry m_cameraTiltPos;
  private double m_currentpos;

  // predefined camera tilt angles
  public static final double TILT_SHELFPICKUP_POS = 0.61;
  public static final double TILT_FLOORPICKUP_POS = 0.20;


  /** Creates a new CameraTilt. */
  public CameraTilt() {
    initializeShuffleboard();
  
    // set default camera angle
    setPosition(TILT_SHELFPICKUP_POS);
  }

  @Override
  public void periodic() {
    // go ahead and set camera angle
    m_servo.set(m_currentpos);

    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  // sets camera tilt to desired angle Position from 0 to 1
  public void setPosition(double value)
  {
      // limit value to allowable range by camera
      if (value > 0.75)  
        value = 0.75;
      if (value < 0.0)
        value = 0.0;

      // set camera value
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

    // used for testing purposes only
    // m_Test= Tab.add("Test", 0.5)
    // .withPosition(3, 1)
    // .withSize(3, 1)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", 0.0, "max", 1.0))
    // .getEntry();
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_cameraTiltPos.setDouble(getAngle());
  }
}
