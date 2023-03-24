// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.OI;
import frc.robot.RobotMap;

public class Grabber extends SubsystemBase {
 
  // subsystem shuffleboard controls
  private GenericEntry m_MotorCurrent; 
  private GenericEntry m_MotorVoltage;
  private GenericEntry m_GrabberPos;
  private GenericEntry m_MotorSpeed;
  private GenericEntry m_SensorDistance;
  public GenericEntry m_Volts;
  private GenericEntry m_TargetAreaHigh;
  private GenericEntry m_TargetAreaMid;


  // spark max motor
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_PIDController;

  // timer to use when open/closing grabber
  private Timer m_timer;

  // grabber motor disable
  private boolean m_enabled;

  // grabber motor speed (rpm)
  private double GrabberMotorSpeed = 6000.0;

  // grabber motor current limit (amps) (must be integer value)
  private int GrabberMotorCurrentLimitStall = 8;
  private int GrabberMotorCurrentLimitFree = 8;

  // function to open and close gripper
  //public enum GrabberPos {
  //  Close,
  //  Open
  //};

  private boolean Open;

  // create range sensor
  AnalogInput m_sensor;

  /** Creates a new Grabber. */
  public Grabber() {

    // create shuffleboard
    initializeShuffleboard();

    // create revmax spark object, set factory defaults
    m_motor = new CANSparkMax(RobotMap.CANID.GRABBER_MOTOR, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    // set motor current limit - stall limit, free limit (in amps)
    m_motor.setSmartCurrentLimit(GrabberMotorCurrentLimitStall, GrabberMotorCurrentLimitFree);

    // set motor PID controller gains
    m_PIDController = m_motor.getPIDController();
    m_PIDController.setP(0.0001);
    m_PIDController.setI(0.0000001);
    m_PIDController.setIMaxAccum(0.05, 0);


    // initially turn off motor
    m_motor.setVoltage(0);
    m_motor.stopMotor();

    // create timer and reset
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();

    Disable();

    // set up range sensor - set ADC to 250 kS/s, and set analog input to oversample by 32 (2^5)
    AnalogInput.setGlobalSampleRate(250000.0);
    m_sensor = new AnalogInput(0);
    m_sensor.setOversampleBits(5);

  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
    if (OI.closeGripper()){
      setClose();
    }

    // update shuffleboard values
    updateShuffleboard();

    // get time into mode
    double t = m_timer.get();
    
    // if motor is enabled, then set voltage according to mode
    if (m_enabled)
    {
      if (Open){
        if (t<1) {
          // if motor is in close mode, apply full effort for limited time, after which, reduce to 1V
            m_PIDController.setReference(GrabberMotorSpeed, CANSparkMax.ControlType.kVelocity);
        } else {
            m_PIDController.setIAccum(0.0);
            m_motor.setVoltage(1); 
        }
      } else {
        if (t <1.5) {
          m_PIDController.setReference(-(GrabberMotorSpeed/4), CANSparkMax.ControlType.kVelocity);
        } else { 
          m_PIDController.setIAccum(0.0); m_motor.setVoltage(0);  
      }
      }
    }
    else
      // we are disabled - turn motor off
      m_motor.setVoltage(0.0);

  }


  // -------------------- Open / Close Methods --------------------

  // sets target position of arm
  public void setOpen ()
  {
    // set our position and restart timer
    m_timer.restart();
    
    // enable grabber motor
    Enable();

    Open = true;
  }

  public void setClose () {
    // set our position and restart timer
    m_timer.restart();
    
    // enable grabber motor
    Enable();

    Open = false;
  }

  // disable / enable grabber motor
  public void Disable()
  {  m_enabled = false; }

  public void Enable()
  { m_enabled = true; }


  // get sensor distance (mmm)
  // returns voltage of GP2Y0A41SK0F sensor
  public double GetSensorDistance()
  {
    return m_sensor.getAverageVoltage();
  }


  // return target area for high cone target
  public double GetTargetAreaHigh()
  {
    return m_TargetAreaHigh.getDouble(0.3);
  }

  // return target area for mid cone target
  public double GetTargetAreaMid()
  {
    return m_TargetAreaMid.getDouble(0.5);
  }



  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Grabber");

    // camera target information
    ShuffleboardLayout l1 = Tab.getLayout("Grabber", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_GrabberPos= l1.add("Grabber Pos", 0.0).getEntry();
    m_MotorCurrent = l1.add("Current(A)", 0.0).getEntry(); 
    m_MotorVoltage = l1.add("Volts(V)", 0.0).getEntry();
    m_MotorSpeed = l1.add("Speed(rpm)", 0.0).getEntry();
    m_SensorDistance = l1.add("Sensor Volts", 0.0).getEntry();
    
    m_Volts = Tab.addPersistent("Volts", 1.50)
    .withPosition(1, 0)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.00, "max", 2.5))
    .getEntry();

    m_TargetAreaHigh = Tab.addPersistent("Target Area High", 0.2)
    .withPosition(1, 2)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 1.0))
    .getEntry();

    m_TargetAreaMid = Tab.addPersistent("Target Area Mid", 0.5)
    .withPosition(1, 3)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 1.0))
    .getEntry();
  }


  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {

    // update motor voltage and current
    m_MotorCurrent.setDouble(m_motor.getOutputCurrent());
    m_MotorVoltage.setDouble(m_motor.getAppliedOutput()*12.0);
    m_MotorSpeed.setDouble(m_motor.getEncoder().getVelocity());

    // update distance
    m_SensorDistance.setDouble(GetSensorDistance());
  }

}