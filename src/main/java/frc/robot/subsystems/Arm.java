// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class Arm extends SubsystemBase {
//public class Arm extends ProfiledPIDSubsystem {
  // Arm Gear Ratios
  // This is the gearbox ratio at the motor
  static final double GEARBOX_RATIO = 80;
  // This is the chain driven gear ratio of the first arm section connected to the upright section
  static final double MID_ARM_RATIO = 2.25;
  // This is the chain driven gear ration of the second arm section connected to the first arm section and grabber
  // It's not needed as of yet, but could come into play with the feed forward loop later on
  static final double END_ARM_RATIO = 1.81818181;
  
  static final double ARM_RATIO = GEARBOX_RATIO * MID_ARM_RATIO;
  // This ARM_RATIO below is just for testing on the bench arm
  //static final double ARM_RATIO = 81;

  // encoder pulse per revolution
  static final double ENC_PULSE_PER_REV = 2048.0;

  // arm motor unit conversion factors
  static final double ENCODERPULSE_TO_DEG = (1.0 / ENC_PULSE_PER_REV) * (1.0 / ARM_RATIO) * 360.0;
  static final double DEG_TO_ENCODERPULSE = 1.0 / ENCODERPULSE_TO_DEG;

  // limit arm speed to this rotational speed
  static final double MAX_VELOCITY_DEG_PER_SECOND = 110;

  // shuffboard entries - used to display arm data
  private GenericEntry m_ArmCanCoderPos;
  private GenericEntry m_ArmCanCoderRawPos;

  private GenericEntry m_ArmPositionFB;
  private GenericEntry m_ArmMotorPosFB;

  private GenericEntry m_ArmPositionSP;
  private GenericEntry m_ArmMotorPositionSP;

  //private GenericEntry m_SetSelMP;
  //private GenericEntry m_GetSelMP;

  private GenericEntry m_ArmEnabledFB;
  private GenericEntry m_ArmForwardLimit;
  private GenericEntry m_ArmReverseLimit;

  
  // Arm Position in degrees for the end arm section relative to the mid arm section
  double m_EndArmPositionDeg;

  // Arm Position in degrees for the mid arm section relative to the fixed arm upright
  double m_MidArmPositionDeg;

  // Arm Position Setpoint in degrees while in position control
  double m_ArmPositionSetpoint;

  // Limit arm positions in degrees - degree limits in the range of 30 - 330 degrees
  // The arm zero degree mark will be when the arm is pointing directly down and the angle increases towards the pickup area and further increases to the drop off area.
  // It is important to ensure ensure the zero value of the arm position is not near either of the MIN or MAX arm position limits.
  static final double MIN_MID_ARM_POS_DEG = 70;
  static final double MAX_MID_ARM_POS_DEG = 262;

  public static final double PICKUP_DEG = 92;
  public static final double LOW_DEG = 107;
  public static final double STOW_DEG = 146;  // was 120
  public static final double MID_DEG = 210.2;
  public static final double PICKUP_SHELF_DEG = 213.0;
  public static final double HIGH_DEG = 260;


  // Arm Cancoder position offset - The angle of the cancoder reported value when the arm is pointing straight down.
  // for testing on the bench unit, simply set it to zero because we don't have a cancoder
  private final double m_ArmCanCoderOffsetDeg = - 40.0;

  // create CANCoder sensor objects
  private CANCoder m_ArmCanCoder;

  // create motor objects
  private TalonFX m_ArmMotor;

  private boolean m_ArmEnabled;


  /** Creates a new Arm. */
  /** Class Constuctor */
  public Arm() {

    m_ArmEnabled = false;
    // create subsystem shuffle board page
    initializeShuffleboard();

    // create CANCoder objects - set absolute range of 0 - 360 deg
    m_ArmCanCoder = new CANCoder(RobotMap.CANID.ARM_CANCODER);
    m_ArmCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    
    // create motors and initalize to factory default
    m_ArmMotor = new TalonFX(RobotMap.CANID.ARM_MOTOR);

    InitializeArm();

  }

  public void InitializeArm(){
    m_ArmMotor.configFactoryDefault();
    m_ArmMotor.configNeutralDeadband(0.001);

    //m_ArmMotor.setNeutralMode(NeutralMode.Coast);
    m_ArmMotor.setNeutralMode(NeutralMode.Brake);
      
    // set steering motor closed loop control gains
    m_ArmMotor.config_kP(0, 0.015, 0);
    m_ArmMotor.config_kI(0, 0.0001, 0);  // 0.0001?
    m_ArmMotor.config_kD(0, 0.05, 0);
    
    // set integration zone and limiter - must haves for using integral gain!
    m_ArmMotor.config_IntegralZone(0, 10.0*DEG_TO_ENCODERPULSE);
    m_ArmMotor.configMaxIntegralAccumulator(0, 1000.0*DEG_TO_ENCODERPULSE);

    // set arm motor peak output and closed loop ramp rate
    m_ArmMotor.configClosedLoopPeakOutput(0,0.65);
    m_ArmMotor.configClosedloopRamp(0.6);
  
    // The other code already is supposed to do this, but keep this as a backup
    // Note the values are set to plus/minus 5 degrees beyond the software limits.
    m_ArmMotor.configReverseSoftLimitThreshold((MIN_MID_ARM_POS_DEG - 5)*DEG_TO_ENCODERPULSE);
    m_ArmMotor.configReverseSoftLimitEnable(true);
    m_ArmMotor.configForwardSoftLimitThreshold((MAX_MID_ARM_POS_DEG + 5)*DEG_TO_ENCODERPULSE);
    m_ArmMotor.configForwardSoftLimitEnable(true);
    m_ArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    m_ArmMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

    // initialize encoders according to CANCoder positions
    ResetArmEncoders();
  }

  
  // seed the encoder value of the arm motor based on CANcoder and alignment position.
  public void ResetArmEncoders() {

    m_ArmMotor.setSelectedSensorPosition((m_ArmCanCoder.getAbsolutePosition()-(m_ArmCanCoderOffsetDeg)) * DEG_TO_ENCODERPULSE, 0, 0);
 
    GetArmPositions();
    // Do not enable this next line of code (The previous lines show in testing that the getselectedmotorposition will not always return the previous setselectedmotorposition)
    //m_ArmPositionSetpoint = m_MidArmPositionDeg;

    m_ArmPositionSetpoint = (m_ArmCanCoder.getAbsolutePosition()-(m_ArmCanCoderOffsetDeg));

  }


// Enable (or disable) Arm Movement.
public void SetEnableArm(boolean Enable) {

  // if the Arm Cancoder is not responding, do not allow the arm to enable.
  if ((Enable) && (m_ArmCanCoder.getAbsolutePosition() == 0.0)) {return;}
  
  // Re Initialize on Enable
  if (Enable) {InitializeArm();}

  m_ArmEnabled = Enable;
  if (!m_ArmEnabled) {
    m_ArmMotor.set(ControlMode.Disabled, 0.0);
  }
}


  private int updateCounter=0;
  @Override
  public void periodic() {

    GetArmPositions();

    // update shuffle board values - update at reduced 5Hz rate to save CPU cycles
    updateCounter+=1;
    if (updateCounter>=10)
    { updateCounter=0; updateShuffleboard(); }
    else if (updateCounter<0)
      updateCounter=0;
  }



// Set Arm Speed in deg/s - (i.e. drive arm up or down with positive or negative speed)
// This is done by converting the speed input to a position setpoint for the arm.
public boolean ArmSpeed_PosCtrl(double speed) {

  if (!m_ArmEnabled) {return false;}
  if (m_ArmMotor.getSensorCollection().isFwdLimitSwitchClosed() == 0.0) {return false;}
  if (m_ArmMotor.getSensorCollection().isRevLimitSwitchClosed() == 0.0) {return false;}

  // arm position delta = arm speed in deg/second * time delta
  // arm position target = current arm position + (arm speed in deg/second * time delta)
  double TargetPosition = m_ArmPositionSetpoint + (speed * 0.02);

  //  This will move the arm to within a valid range if it is outside of it's valid range.
  //  if (TargetPosition < MIN_MID_ARM_POS_DEG) {TargetPosition = MIN_MID_ARM_POS_DEG;}
  //  if (TargetPosition > MAX_MID_ARM_POS_DEG) {TargetPosition = MAX_MID_ARM_POS_DEG;}

  // This will not allow the arm to move further outside of it's range but allow it to move towards it's valid range.
  if ((TargetPosition < MIN_MID_ARM_POS_DEG) && (TargetPosition < m_ArmPositionSetpoint)) {TargetPosition = m_ArmPositionSetpoint;}
  if ((TargetPosition > MAX_MID_ARM_POS_DEG) && (TargetPosition > m_ArmPositionSetpoint)) {TargetPosition = m_ArmPositionSetpoint;}
  

  return SetArmPosition(TargetPosition);
}


// Set Arm Position deg
   public boolean SetArmPosition(double PosDeg) {
    if (!m_ArmEnabled) {return false;}
    if (m_ArmMotor.getSensorCollection().isFwdLimitSwitchClosed() == 0.0) {return false;}
    if (m_ArmMotor.getSensorCollection().isRevLimitSwitchClosed() == 0.0) {return false;}

    // Do not set the arm position if an invalid arm position is selected.
    if ((PosDeg >= MIN_MID_ARM_POS_DEG) && (PosDeg <= MAX_MID_ARM_POS_DEG)) {
      m_ArmPositionSetpoint = PosDeg;
      m_ArmMotor.set(ControlMode.Position, m_ArmPositionSetpoint*DEG_TO_ENCODERPULSE, DemandType.ArbitraryFeedForward, 0.0);

      return true;
    } else {
      return false;
    }
  }

// Get Arm Enabled Status
public boolean GetArmEnabled() {
  return m_ArmEnabled;
}


// Get Arm Position in degrees 
public double GetArmPosition() {
  //GetArmPositions();  // This was moved to periodic
  //Do we need to add info to get arm position in here?
  return m_MidArmPositionDeg;
}

// Get arm position according to CANcoder in degrees
public double GetArmPositionCANCoder() {
  return (m_ArmCanCoder.getAbsolutePosition()-m_ArmCanCoderOffsetDeg)%360;
} 

// Get Arm Position in degrees
private void GetArmPositions() {
    // determine the arm position with the mod operator but note that it will can return between -360 and 360 so need to add a rotation to make it in the 0-360 range.
  double MidArmPositionDeg = (m_ArmMotor.getSelectedSensorPosition()*ENCODERPULSE_TO_DEG)%360;
  if (MidArmPositionDeg < 0.0) {
    m_MidArmPositionDeg = MidArmPositionDeg + 360.0;
  }else{
    m_MidArmPositionDeg = MidArmPositionDeg;
  }
}


public void EnableFast(boolean enable)
{
  if (enable)
  {
    m_ArmMotor.configClosedLoopPeakOutput(0,0.65);
    m_ArmMotor.configClosedloopRamp(0.5);
  }
  else
  {
    m_ArmMotor.configClosedLoopPeakOutput(0,0.65);
    m_ArmMotor.configClosedloopRamp(0.6);
  }

}


  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create arm page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Arm");

    // create button/command to enable/disable arm 
    Tab.add("Arm Enable Disable", new InstantCommand(()->{SetEnableArm(!m_ArmEnabled);}))
    .withPosition(0,0)
    .withSize(2, 1);
    
    // create button/command to reset arm 
    Tab.add("Arm Reset", new InstantCommand(()->{InitializeArm();}))
    .withPosition(0,1)
    .withSize(2, 1);

    // create controls to arm motor and position
    ShuffleboardLayout l1 = Tab.getLayout("CanCoder", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(1, 5);
    m_ArmCanCoderPos = l1.add("CanCoder Deg", 0.0).getEntry();
    m_ArmCanCoderRawPos = l1.add("CanCoder Raw Deg", 0.0).getEntry();

    ShuffleboardLayout l2 = Tab.getLayout("Position FB", BuiltInLayouts.kList);
    l2.withPosition(3, 0);
    l2.withSize(1, 5);
    m_ArmPositionFB = l2.add("Arm FB Deg", 0.0).getEntry();
    m_ArmMotorPosFB = l2.add("Motor FB Cnt", 0.0).getEntry();

    ShuffleboardLayout l3 = Tab.getLayout("Position SP", BuiltInLayouts.kList);
    l3.withPosition(4, 0);
    l3.withSize(1, 5);
    m_ArmPositionSP= l3.add("Arm SP Deg", 0.0).getEntry();
    m_ArmMotorPositionSP= l3.add("Motor SP Cnt", 0.0).getEntry();

    ShuffleboardLayout l4 = Tab.getLayout("Arm Enabled", BuiltInLayouts.kList);
    l4.withPosition(5, 0);
    l4.withSize(1, 5);
    m_ArmEnabledFB= l4.add("Arm Enabled", false).getEntry();
    m_ArmForwardLimit= l4.add("Forward Limit OK", false).getEntry();    
    m_ArmReverseLimit= l4.add("Reverse Limit OK", false).getEntry();
  }

  /** Update subsystem shuffle board page with current values */
  private void updateShuffleboard() {
    // update CANCoder position values (degrees)
    m_ArmCanCoderPos.setDouble((m_ArmCanCoder.getAbsolutePosition()-m_ArmCanCoderOffsetDeg)%360);
    m_ArmCanCoderRawPos.setDouble(m_ArmCanCoder.getAbsolutePosition());

    // update arm motor position feedback values
    m_ArmPositionFB.setDouble(m_MidArmPositionDeg);
    m_ArmMotorPosFB.setDouble(m_ArmMotor.getSelectedSensorPosition());

    // update arm motor position setpoint values
    m_ArmPositionSP.setDouble(m_ArmPositionSetpoint);
    m_ArmMotorPositionSP.setDouble(m_ArmMotor.getClosedLoopTarget());

    // update arm enabled boolean indicator
    m_ArmEnabledFB.setBoolean(m_ArmEnabled);
    
    // update limit switch indicators
    m_ArmForwardLimit.setBoolean(m_ArmMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1.0);
    m_ArmReverseLimit.setBoolean(m_ArmMotor.getSensorCollection().isRevLimitSwitchClosed() == 1.0);
  }


}
