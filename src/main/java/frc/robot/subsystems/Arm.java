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
import com.ctre.phoenix.schedulers.SequentialScheduler;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.RobotMap;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


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
  private GenericEntry m_ArmMotorPosDeg;
  private GenericEntry m_ArmMotorPos;
  private GenericEntry m_ArmSpeedSP;
  private GenericEntry m_ArmSpeedFB;


  // Arm Position in degrees for the end arm section relative to the mid arm section
  double m_EndArmPositionDeg;

  // Arm Position in degrees for the mid arm section relative to the fixed arm upright
  double m_MidArmPositionDeg;

  // Arm Cancoder position offset - The angle of the cancoder reported value when the arm is pointing straight down.
  double m_ArmCanCoderOffsetDeg;

  // Limit arm positions in degrees - degree limits in the range of 30 - 330 degrees
  // The arm zero degree mark will be when the arm is pointing directly down and the angle increases towards the pickup area and further increases to the drop off area.
  // It is important to ensure ensure the zero value of the arm position is not near either of the MIN or MAX arm position limits.
  static final double MIN_MID_ARM_POS_DEG = 80;
  static final double MAX_MID_ARM_POS_DEG = 220;
  
  // create CANCoder sensor objects
  private CANCoder m_ArmCanCoder;

  // create motor objects
  private TalonFX m_ArmMotor;

  private final ArmFeedforward m_feedforward =
  new ArmFeedforward(0.0,0.0,0.0,0.0);
  //  ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
//new ArmFeedforward(
  //   ArmConstants.kSVolts, ArmConstants.kGVolts,
  //  ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Creates a new Arm. */
  /** Class Constuctor */
/**   public Arm() {
   super(
      new ProfiledPIDController(
          0.1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              6.0,
              6.0)),
      0);
  */
  //m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
  // Start arm at rest in neutral position
//  setGoal(ArmConstants.kArmOffsetRads);

/** Creates a new Arm. */
  /** Class Constuctor */
  public Arm() {

    // create CANCoder objects - set absolute range of +/-180deg
    m_ArmCanCoder = new CANCoder(RobotMap.CANID.ARM_CANCODER);
    m_ArmCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    
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

    // This is simply here for arm testing, can be removed later on if we see fit, or leave it if it's too powerfull
        m_ArmMotor.configClosedLoopPeakOutput(0,0.2);
  }

  
  // seed the encoder value of the arm motor based on CANcoder and alignment position
  public void ResetArmEncoders() {
    
    // Add this in once the robot is built with an appropriate value

    // for testing on the bench unit, simply set it to zero because we don't have a cancoder
    //m_ArmCanCoderOffsetDeg = 0.0;
    m_ArmCanCoderOffsetDeg = -49;

    m_ArmMotor.setSelectedSensorPosition((m_ArmCanCoder.getAbsolutePosition()-(m_ArmCanCoderOffsetDeg)) * DEG_TO_ENCODERPULSE, 0, 0);

  }

  private int updateCounter=0;
  @Override
  public void periodic() {

    GetArmPositions();

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

    // This code will set the speed of the arm to zero as soon as the arm angle goes out of the valid range but only if the target speed is still
    // driving it out of range. If the target speed is telling the arm to go back to the valid range, then it will allow the arm to move that way.
    // Note:  The location of zero degrees is what identifies which way the arm is allowed to move outside of the invalid range of motion.
    if ((m_MidArmPositionDeg <= MIN_MID_ARM_POS_DEG)&&(TargetSpeed < 0.0)){
      m_ArmMotor.set(ControlMode.Velocity, 0.0);
    } else if ((m_MidArmPositionDeg >= MAX_MID_ARM_POS_DEG)&&(TargetSpeed > 0.0)){
      m_ArmMotor.set(ControlMode.Velocity, 0.0);
    } else {
      // go ahead and set motor closed loop target speeds (in encoder pulses per 100ms)
      // The FeedForward is not added in yet, but we could do this and populate it below instead of 0.0 hardcoded
      m_ArmMotor.set(ControlMode.Velocity, TargetSpeed*DEG_TO_ENCODERPULSE*0.1, DemandType.ArbitraryFeedForward, 0.0);
    }
    //Just some test code to set the arm speed
    //m_ArmMotor.set(ControlMode.Velocity, -9220, DemandType.ArbitraryFeedForward,0.0);
  }


// Set Arm Speed in deg/s - (i.e. drive arm up or down with positive or negative speed)
// This is done by converting the speed input to a position setpoint for the arm.
public boolean ArmSpeed_PosCtrl(double speed) {
  // arm position delta = arm speed in deg/second * time delta
  // arm position target = current arm position + (arm speed in deg/second * time delta)
  double TargetPosition = m_MidArmPositionDeg + (speed * 0.02);
  return SetArmPosition(TargetPosition);
}


// Set Arm Position deg
   public boolean SetArmPosition(double PosDeg) {

    // Add code to limit or cancel if an invalid arm position is selected.
    if ((PosDeg > MIN_MID_ARM_POS_DEG) && (PosDeg < MAX_MID_ARM_POS_DEG)) {
      m_ArmMotor.set(ControlMode.Position, PosDeg*DEG_TO_ENCODERPULSE, DemandType.ArbitraryFeedForward, 0.0);
      return true;
    } else {
    return false;
    }
  }

// Get Arm Position in degrees 
public double GetArmPosition() {
  //GetArmPositions();  // This was moved to periodic
  return m_MidArmPositionDeg;
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


  // Add in code here to determine the end arm position if needed....

  // Possibly add in code here to hard stop the arm and not let it start back up again if it goes too far beyond
  // an allowable range so the robot does not beat itslef up.


  /**

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    
    //m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
//    return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
    return 0.0;
  }

 */



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
    m_ArmSpeedSP = l1.add("Arm Speed SP", 0.0).getEntry();
    m_ArmSpeedFB = l1.add("Arm Speed FB", 0.0).getEntry();
    
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // update CANCoder position values (degrees)
    m_ArmCanCoderPos.setDouble((m_ArmCanCoder.getAbsolutePosition()-m_ArmCanCoderOffsetDeg)%360);

    // update arm motor position values (degrees)
    m_ArmMotorPosDeg.setDouble(m_MidArmPositionDeg);

    // update arm motor position values (encoder pulses)
    m_ArmMotorPos.setDouble(m_ArmMotor.getSelectedSensorPosition());

    // update arm motor speed reference (Setpoint) in deg/second    The x10 is because it's the encoder pulses per 100ms
    m_ArmSpeedSP.setDouble(m_ArmMotor.getClosedLoopTarget()*ENCODERPULSE_TO_DEG*10);

    // update arm motor speed reference (Setpoint) in deg/second    The x10 is because it's the encoder pulses per 100ms
    m_ArmSpeedFB.setDouble(m_ArmMotor.getSelectedSensorVelocity()*ENCODERPULSE_TO_DEG*10);

  }

}