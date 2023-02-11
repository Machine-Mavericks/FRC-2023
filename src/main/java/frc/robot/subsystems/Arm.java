// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.RobotMap;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Arm extends SubsystemBase {

  // Arm Gear Ratios
  // This is the gearbox ratio at the motor
  static final double GEARBOX_RATIO = 16;
  // This is the chain driven gear ratio of the first arm section connected to the upright section
  static final double MID_ARM_RATIO = 2;
  // This is the chain driven gear ration of the second arm section connected to the first arm section and grabber
  static final double END_ARM_RATIO = 1.666666;
  
  //static final double ARM_RATIO = GEARBOX_RATIO * MID_ARM_RATIO * END_ARM_RATIO;
  // This ARM_RATIO below is just for testing on the bench
  static final double ARM_RATIO = 90;

  // encoder pulse per revolution
  static final double ENC_PULSE_PER_REV = 2048.0;

  // arm motor unit conversion factors
  static final double ENCODERPULSE_TO_DEG = (1.0 / ENC_PULSE_PER_REV) * (1.0 / ARM_RATIO) * 360.0;
  static final double DEG_TO_ENCODERPULSE = 1.0 / ENCODERPULSE_TO_DEG;

  // limit arm speed to this rotational speed
  static final double MAX_VELOCITY_DEG_PER_SECOND = 180;

  // shuffboard entries - used to display arm data
  private GenericEntry m_ArmCanCoderPos;
  private GenericEntry m_ArmMotorPosDeg;
  private GenericEntry m_ArmMotorPos;

  // create CANCoder sensor objects
  private CANCoder m_ArmCanCoder;

  // create motor objects
  private TalonFX m_ArmMotor;

  /** Creates a new Arm. */
  /** Class Constuctor */
  public Arm() {
    
    // create CANCoder objects - set absolute range of +/-180deg
    m_ArmCanCoder = new CANCoder(RobotMap.CANID.ARM_CANCODER);
    m_ArmCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    
    // create motors and initalize to factory default
    m_ArmMotor = new TalonFX(RobotMap.CANID.ARM_MOTOR);
    m_ArmMotor.configFactoryDefault();
    m_ArmMotor.configNeutralDeadband(0.001);
    m_ArmMotor.setNeutralMode(NeutralMode.Coast);
    
    // set steering motor closed loop control gains
    m_ArmMotor.config_kP(0, 0.1, 0);
    m_ArmMotor.config_kI(0, 0.000, 0);
    m_ArmMotor.config_kD(0, 0.05, 0);
   
    // initialize encoders of each steer motor according to CANCoder positions
    ResetArmEncoders();

    // create subsystem shuffle board page
    initializeShuffleboard();
  }

  
  // seed the encoder value of the arm motor based on CANcoder and alignment position
  public void ResetArmEncoders() {
    
    // Add this in once the robot is built with an appropriate value
    // m_ArmMotor.setSelectedSensorPosition((m_ArmCanCoder.getAbsolutePosition()-(-158.99)) * DEG_TO_ENCODERPULSE, 0, 0);
    // for testing on the bench unit, simply set it to zero because we don't have a cancoder
    m_ArmMotor.setSelectedSensorPosition(0.0 * DEG_TO_ENCODERPULSE,0,0);
  
  }

  private int updateCounter=0;
  @Override
  public void periodic() {
    
    // update shuffle board values - update at reduced 5Hz rate to save CPU cycles
    updateCounter+=1;
    if (updateCounter>=5)
    { updateCounter=0; updateShuffleboard(); }
    else if (updateCounter<0)
      updateCounter=0;
  }

  
  // Set Arm Speed in deg/s - (i.e. drive arm up or down with positive or negative speed)
 
  public void ArmSpeed(double speed) {
  
    // limit arm speed
    double TargetSpeed = speed;
    if (TargetSpeed > MAX_VELOCITY_DEG_PER_SECOND) {TargetSpeed = MAX_VELOCITY_DEG_PER_SECOND;}

    
    /*
    // ---------- Angle Determination for LF Swerve
    
    // adder used to determine angle depending on direction of swerve drive
    double adder1=0.0;

    // get LF motor's current drive direction. If currently in reverse:
    // a) consider its angle to be 180deg more than its sensor shows; and
    // b) set direction flag to reverse
    if (m_LFDriveMotor.getClosedLoopTarget()<0.0)
      { adder1 = 180.0; LFDriveDir = -1.0; }
    
    // get current angle of swerve (in deg)
    double LFCurrentAngleDeg = m_LFSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG;
    
    // determine smallest angle to turn swerve to get to desired angle
    double LFAngleDiff = Utils.AngleDifference(LFCurrentAngleDeg%360.0, adder1+m_states[0].angle.getDegrees());
    
    // to minimize turning, it may be easier to reverse drive, and turn by smaller angle
    if (LFAngleDiff<-90.0)
      { LFDriveDir *= -1.0; LFAngleDiff+=180.0; }
    else if (LFAngleDiff>90)
      { LFDriveDir *= -1.0; LFAngleDiff-=180.0; }
    
    // set angle of swerve drive
    m_LFSteerMotor.set(ControlMode.Position, (LFCurrentAngleDeg + LFAngleDiff)*DEG_TO_ENCODERPULSE);
    */

    
    // ---------- Set Arm Motor Speed
    // go ahead and set motor closed loop target speeds (in encoder pulses per 100ms)
    m_ArmMotor.set(ControlMode.Velocity, TargetSpeed*DEG_TO_ENCODERPULSE*0.1, DemandType.ArbitraryFeedForward, 0.0);
    //m_ArmMotor.set(ControlMode.Velocity, -9220, DemandType.ArbitraryFeedForward,0.0);
        
  }


// Set Arm Position deg
   public void ArmPosition(double PosDeg) {
  
    
  }



  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create arm page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Arm");

    // create controls to arm motor and position
    ShuffleboardLayout l1 = Tab.getLayout("Arm Motor and Position", BuiltInLayouts.kList);
    l1.withPosition(3, 0);
    l1.withSize(1, 5);
    m_ArmCanCoderPos = l1.add("CanCoder Deg", 0.0).getEntry();
    m_ArmMotorPosDeg = l1.add("Motor Pos Deg", 0.0).getEntry();
    m_ArmMotorPos = l1.add("Motor Pos'n", 0.0).getEntry();
    
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // update CANCoder position values (degrees)
    m_ArmCanCoderPos.setDouble(m_ArmCanCoder.getAbsolutePosition());

    // update arm motor position values (degrees)
    m_ArmMotorPosDeg.setDouble((m_ArmMotor.getSelectedSensorPosition()*ENCODERPULSE_TO_DEG)%360);

    // update steer motor position values (encoder pulses)
    m_ArmMotorPos.setDouble(m_ArmMotor.getSelectedSensorPosition());

  }

}
