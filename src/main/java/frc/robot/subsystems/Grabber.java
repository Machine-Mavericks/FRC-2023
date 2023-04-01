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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.Rev2mDistanceSensor.Port;
//import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
//import com.revrobotics.Rev2mDistanceSensor.Unit;

import frc.robot.OI;
import frc.robot.RobotMap;

public class Grabber extends SubsystemBase {
 
  // subsystem shuffleboard controls
  private GenericEntry m_MotorCurrent; 
  private GenericEntry m_MotorVoltage;
  private GenericEntry m_GrabberPos;
  private GenericEntry m_MotorSpeed;
  private GenericEntry m_SensorDistance;
  private GenericEntry m_I2CDistance;
  public GenericEntry m_Volts;
  private GenericEntry m_ConeTargetAreaHigh;
  private GenericEntry m_ConeTargetAreaMid;
  private GenericEntry m_CubeTargetAreaHigh;
  private GenericEntry m_CubeTargetAreaMid;
  private GenericEntry m_ObjectGrabbed;
  private GenericEntry m_UltrasonicDistance;
  private GenericEntry m_UltrasonicDistSelect;



  // spark max motor
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_PIDController;

  // timer to use when open/closing grabber
  private Timer m_timer;

  // grabber motor disable
  private boolean m_enabled;

  // grabber motor speed (rpm)
  private double GrabberMotorSpeedLoad = 16000.0;
  private double GrabberMotorSpeedDrop = 1500.0;

  // grabber motor current limit (amps) (must be integer value)
  private int GrabberMotorCurrentLimitStall = 8;
  private int GrabberMotorCurrentLimitFree = 8;

  private boolean Open;

  // create range sensor
  private AnalogInput m_sensor;

  // grabber sensor
 private  DigitalInput m_grabsensor;

  // ultrasonic distance sensor
  private Ultrasonic m_ultrasonicsensor;

  //private Rev2mDistanceSensor I2CSensor;

  // i2c for for TF Luna Sensor
  private I2C m_i2cPort;

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

    // set up I2C sensor
    //I2CSensor = new Rev2mDistanceSensor(Port.kMXP,Unit.kInches,RangeProfile.kDefault);
    //I2CSensor.setAutomaticMode(true);
    //I2CSensor.setEnabled(true);

    // set up range sensor - set ADC to 250 kS/s, and set analog input to oversample by 32 (2^5)
    AnalogInput.setGlobalSampleRate(100000.0);
    m_sensor = new AnalogInput(0);
    m_sensor.setOversampleBits(5);

    // set up digital switch - to detect when something is grabbed
    m_grabsensor = new DigitalInput(0);
    
    // ultrasonic distance sensor
    m_ultrasonicsensor = new Ultrasonic(3, 4);
    m_ultrasonicsensor.setAutomaticMode(true);
    m_ultrasonicsensor.setEnabled(true);
    
    // set up TF luna 
    //i2c


  }

  // This method will be called once per scheduler run
  double stalltimer=0.0;
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
      // is motor stalling because it has a cone/cube in it? if so, add to time
      if (m_motor.getOutputCurrent()>20.0)
        stalltimer +=0.02;
      if (GetTargetGrabbedStatus())
        stalltimer = 1.0;

      if (Open){   
        // apply full speed for 5s or until motor stalled for 1.0s.
        if (t<5.0 && stalltimer<1.0) {
          // if motor is in close mode, apply full effort for limited time, after which, reduce to 1V
            m_PIDController.setReference(GrabberMotorSpeedLoad, CANSparkMax.ControlType.kVelocity);
        } else {
            m_PIDController.setIAccum(0.0);
            m_motor.setVoltage(1); 
        }
      } else {
        if (t <1.5) {
          m_PIDController.setReference(-(GrabberMotorSpeedDrop), CANSparkMax.ControlType.kVelocity);
        } else { 
          m_PIDController.setIAccum(0.0); m_motor.setVoltage(0);  
      }
      }
    }
    else
      // we are disabled - turn motor off
      { m_motor.setVoltage(0.0); stalltimer=0.0; }

  }


  // -------------------- Open / Close Methods --------------------

  // sets target position of arm
  public void setOpen ()
  {
    // set our position and restart timer
    m_timer.restart();
    
    // reset motor stall timer
    stalltimer = 0.0;
    
    // enable grabber motor
    Enable();

    Open = true;
  }

  public void setClose () {
    // set our position and restart timer
    m_timer.restart();
    
    // reset motor stall timer
    stalltimer = 0.0;

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
  public double GetConeTargetAreaHigh()
  { return m_ConeTargetAreaHigh.getDouble(0.3); }

  // return target area for mid cone target
  public double GetConeTargetAreaMid()
  { return m_ConeTargetAreaMid.getDouble(0.5); }

  // return target area for high cone target
  public double GetCubeTargetAreaHigh()
  { return m_CubeTargetAreaHigh.getDouble(0.3); }

  // return target area for mid cone target
  public double GetCubeTargetAreaMid()
  { return m_CubeTargetAreaMid.getDouble(0.5); }

  // get ultrasonic distance selection
  public double GetUltrasonicDistSelection()
  {
    return m_UltrasonicDistSelect.getDouble(24.0);
  }

  // get ultrasonic snesor distance (in)
  public double GetUltrasonicDistance()
  {
    return m_ultrasonicsensor.getRangeInches();
  }
  // returns true if switch says cone is grabbed
  public boolean GetTargetGrabbedStatus()
  {
    return !m_grabsensor.get();
  }



  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Grabber");

    // camera target information
    ShuffleboardLayout l1 = Tab.getLayout("Grabber", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 5);
    m_GrabberPos= l1.add("Grabber Pos", 0.0).getEntry();
    m_MotorCurrent = l1.add("Current(A)", 0.0).getEntry(); 
    m_MotorVoltage = l1.add("Volts(V)", 0.0).getEntry();
    m_MotorSpeed = l1.add("Speed(rpm)", 0.0).getEntry();
    m_SensorDistance = l1.add("Sensor Volts", 0.0).getEntry();
    m_UltrasonicDistance = l1.add("Ultrasonic Dist(in)", 0.0).getEntry();
    m_I2CDistance = l1.add("I2C Dist(in)", 0.0).getEntry();
    
    m_ConeTargetAreaHigh = Tab.addPersistent("Target Area High", 0.2)
    .withPosition(1, 0)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 1.0))
    .getEntry();

    m_ConeTargetAreaMid = Tab.addPersistent("Target Area Mid", 0.5)
    .withPosition(1, 1)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 1.0))
    .getEntry();

    m_CubeTargetAreaHigh = Tab.addPersistent("Cube Area High", 0.2)
    .withPosition(1, 2)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 7.5))
    .getEntry();

    m_CubeTargetAreaMid = Tab.addPersistent("Cube Area Mid", 0.5)
    .withPosition(1, 3)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 7.5))
    .getEntry();

    m_UltrasonicDistSelect = Tab.addPersistent("Ultrasonic Dist Select", 24.0)
    .withPosition(4, 0)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.0, "max", 50.0))
    .getEntry();

    m_Volts = Tab.addPersistent("Volts", 2.25)
    .withPosition(4, 1)
    .withSize(3, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.00, "max", 3.0))
    .getEntry();

    // add sensor
    m_ObjectGrabbed = Tab.add("Cone Grabbed", false).withPosition(4,2).getEntry();

  }


  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {

    // update motor voltage and current
    m_MotorCurrent.setDouble(m_motor.getOutputCurrent());
    m_MotorVoltage.setDouble(m_motor.getAppliedOutput()*12.0);
    m_MotorSpeed.setDouble(m_motor.getEncoder().getVelocity());

    // update distance
    m_SensorDistance.setDouble(GetSensorDistance());

    // update sensor
    m_ObjectGrabbed.setBoolean(GetTargetGrabbedStatus());

    // ultrasonic distance
    m_UltrasonicDistance.setDouble(m_ultrasonicsensor.getRangeInches());

    // i2c distance
    //m_I2CDistance.setDouble(I2CSensor.getRange());

  }

}