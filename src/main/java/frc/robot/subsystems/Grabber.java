// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;

public class Grabber extends SubsystemBase {
 
  // subsystem shuffleboard controls
  private GenericEntry m_MotorCurrent; 
  private GenericEntry m_MotorVoltage;
  private GenericEntry m_GrabberPos;
 
  // spark max motor
  private CANSparkMax m_motor;

  // timer to use when open/closing grabber
  private Timer m_timer;

  // grabber motor disable
  private boolean m_enabled;

  // function to open and close gripper
  public enum GrabberPos {
    Close,
    Open
  };
  GrabberPos m_targetPos;

  /** Creates a new Grabber. */
  public Grabber() {

    // create shuffleboard
    initializeShuffleboard();

    // create revmax spark object, set factory defaults
    m_motor = new CANSparkMax(RobotMap.CANID.GRABBER_MOTOR, MotorType.kBrushed);
    m_motor.restoreFactoryDefaults();

    // set motor current limit (in amps)
    m_motor.setVoltage(0);
    m_motor.stopMotor();

    // create timer and reset
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();

    Disable();

  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
  
    // update shuffleboard values
    updateShuffleboard();

    // get time into mode
    double t = m_timer.get();

    // if motor is enabled, then set voltage according to mode
    if (m_enabled)
    {
      // control motor based on target position
      if (m_targetPos==GrabberPos.Close)
      {
        // if motor is in close mode, apply full effort for 0.8s, after which, reduce to 0V
        if (t <0.8)
          m_motor.setVoltage(-12.0);
        else
        m_motor.setVoltage(0.0);
      }
      else
      {
        if (t <0.6)
          m_motor.setVoltage(12.0);
        else
          m_motor.setVoltage(0);
      }
    }

    else
      // we are disabled - turn motor off
      m_motor.setVoltage(0.0);

  }

  // -------------------- Open / Close Methods --------------------

  // sets target position of arm
  public void setPosition (GrabberPos pos)
  {
    // set our position and restart timer
    m_targetPos = pos;
    m_timer.restart();
    
    // enable grabber motor
    Enable();
  }
  
  // function to alternate grabber position from current state
  // useful to use if single joystick button to be used to open/close gripper
  public void setAlternatePosition()
  {
    if (m_targetPos == GrabberPos.Close)
      setPosition(GrabberPos.Open);
    else
      setPosition(GrabberPos.Close);
  }

  // gets gripepr target position
  public GrabberPos getPosition()
  {
    return m_targetPos;
  }


  // disable / enable grabber motor
  public void Disable()
  {  m_enabled = false; }

  public void Enable()
  { m_enabled = true; }



  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Grabber");

    // camera target information
    ShuffleboardLayout l1 = Tab.getLayout("Grabber", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(1, 4);
    m_GrabberPos= l1.add("Grabber Pos", 0.0).getEntry();
    m_MotorCurrent = l1.add("Current", 0.0).getEntry(); 
    m_MotorVoltage = l1.add("Applied Out", 0.0).getEntry();
    }


    /** Update subsystem shuffle board page with current odometry values */
    private void updateShuffleboard() {
      // update gripper position
      if (m_targetPos==GrabberPos.Open)
        m_GrabberPos.setDouble(0.0);
      else
        m_GrabberPos.setDouble(1.0);
    
      // update motor voltage and current
      m_MotorCurrent.setDouble(m_motor.getOutputCurrent());
      m_MotorVoltage.setDouble(m_motor.getAppliedOutput()*12.0);
    }


}