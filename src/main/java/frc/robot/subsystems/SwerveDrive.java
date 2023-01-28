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
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.RobotMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Utils;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {

  // steering gear ratio for SPS MK4 L1 Swerve
  static final double STEER_RATIO = 12.8;

  // drive gear ratio for SPS MK4 L1 Swerve
  static final double DRIVE_RATIO = 8.14;

  // encoder pulse per revolution
  static final double ENC_PULSE_PER_REV = 2048.0;

  // wheel diameter (m)
  static final double WHEEL_DIAMETER_METERS = 0.1016 * 0.96248;
  
  // drive motor unit conversion factors
  static final double ENCODERPULSE_TO_METERS = (1.0 / ENC_PULSE_PER_REV) * (1.0 / DRIVE_RATIO) * WHEEL_DIAMETER_METERS * Math.PI;
  static final double METERS_TO_ENCODERPULSE = 1.0 / ENCODERPULSE_TO_METERS;
 
  // steer motor unit conversion factors
  static final double ENCODERPULSE_TO_DEG = (1.0 / ENC_PULSE_PER_REV) * (1.0 / STEER_RATIO) * 360.0;
  static final double DEG_TO_ENCODERPULSE = 1.0 / ENCODERPULSE_TO_DEG;

  // theoretical maximum speed of drive (m/s)
  // = <Motor free speed RPM> / 60 / Drive Gear ratio * <Wheel diameter meters> * pi
  //static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0) * (1.0 / DRIVE_RATIO) * WHEEL_DIAMETER_METERS * Math.PI;
  
  // set temporary maximum speed for safe testing purposes (use 1.5 m/s)
  static final double MAX_VELOCITY_METERS_PER_SECOND = 1.50;

  // shuffboard entries - used to display swerve drive data
  private GenericEntry m_LFCanCoderPos;
  private GenericEntry m_RFCanCoderPos;
  private GenericEntry m_LRCanCoderPos;
  private GenericEntry m_RRCanCoderPos;
  private GenericEntry m_LFSteerMotorPos;
  private GenericEntry m_RFSteerMotorPos;
  private GenericEntry m_LRSteerMotorPos;
  private GenericEntry m_RRSteerMotorPos;
  private GenericEntry m_LFSteerMotorTarget;
  private GenericEntry m_RFSteerMotorTarget;
  private GenericEntry m_LRSteerMotorTarget;
  private GenericEntry m_RRSteerMotorTarget;
  private GenericEntry m_LFDriveMotorPos;
  private GenericEntry m_RFDriveMotorPos;
  private GenericEntry m_LRDriveMotorPos;
  private GenericEntry m_RRDriveMotorPos;
  private GenericEntry m_LFDriveMotorTargetSpeed;
  private GenericEntry m_RFDriveMotorTargetSpeed;
  private GenericEntry m_LRDriveMotorTargetSpeed;
  private GenericEntry m_RRDriveMotorTargetSpeed;
  private GenericEntry m_LFDriveMotorSpeed;
  private GenericEntry m_RFDriveMotorSpeed;
  private GenericEntry m_LRDriveMotorSpeed;
  private GenericEntry m_RRDriveMotorSpeed;
  private GenericEntry m_LFDriveMotorVolts;
  private GenericEntry m_RFDriveMotorVolts;
  private GenericEntry m_LRDriveMotorVolts;
  private GenericEntry m_RRDriveMotorVolts;
  private GenericEntry m_LFDriveMotorTemp;
  private GenericEntry m_RFDriveMotorTemp;
  private GenericEntry m_LRDriveMotorTemp;
  private GenericEntry m_RRDriveMotorTemp;
  private GenericEntry m_BattVolts;

  // create CANCoder sensor objects
  private CANCoder m_LFCanCoder;
  private CANCoder m_RFCanCoder;
  private CANCoder m_LRCanCoder;
  private CANCoder m_RRCanCoder;
  // create steer motor objects
  private TalonFX m_LFSteerMotor;
  private TalonFX m_RFSteerMotor;
  private TalonFX m_LRSteerMotor;
  private TalonFX m_RRSteerMotor;
  // create drive motor objects
  private TalonFX m_LFDriveMotor;
  private TalonFX m_RFDriveMotor;
  private TalonFX m_LRDriveMotor;
  private TalonFX m_RRDriveMotor;

  /**The model representing the drivetrain's kinematics */
  public static final double TRACKWIDTH_METERS = 0.6;
  public static final double WHEELBASE_METERS = 0.6;
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS*0.5, WHEELBASE_METERS*0.5),
            // Front right
            new Translation2d(TRACKWIDTH_METERS*0.5, -WHEELBASE_METERS*0.5),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS*0.5, WHEELBASE_METERS*0.5),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS*0.5, -WHEELBASE_METERS*0.5));

  // Swerve module states - contains speed(m/s) and angle for each swerve module
  SwerveModuleState[] m_states;


  /** Creates a new SwerveDrive. */
  /** Class Constuctor */
  public SwerveDrive() {
    // create CANCoder objects - set absolute range of +/-180deg
    m_LFCanCoder = new CANCoder(RobotMap.CANID.LF_CANCODER);
    m_RFCanCoder = new CANCoder(RobotMap.CANID.RF_CANCODER);
    m_LRCanCoder = new CANCoder(RobotMap.CANID.LR_CANCODER);
    m_RRCanCoder = new CANCoder(RobotMap.CANID.RR_CANCODER);
    m_LFCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_RFCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_LRCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_RRCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    // create steer motors and initalize to factory default
    m_LFSteerMotor = new TalonFX(RobotMap.CANID.LF_STEER_MOTOR);
    m_RFSteerMotor = new TalonFX(RobotMap.CANID.RF_STEER_MOTOR);
    m_LRSteerMotor = new TalonFX(RobotMap.CANID.LR_STEER_MOTOR);
    m_RRSteerMotor = new TalonFX(RobotMap.CANID.RR_STEER_MOTOR);
    m_LFSteerMotor.configFactoryDefault();
    m_RFSteerMotor.configFactoryDefault();
    m_LRSteerMotor.configFactoryDefault();
    m_RRSteerMotor.configFactoryDefault();
    m_LFSteerMotor.configNeutralDeadband(0.001);
    m_RFSteerMotor.configNeutralDeadband(0.001);
    m_LRSteerMotor.configNeutralDeadband(0.001);
    m_RRSteerMotor.configNeutralDeadband(0.001);
    m_LFSteerMotor.setNeutralMode(NeutralMode.Coast);
    m_RFSteerMotor.setNeutralMode(NeutralMode.Coast);
    m_LRSteerMotor.setNeutralMode(NeutralMode.Coast);
    m_RRSteerMotor.setNeutralMode(NeutralMode.Coast);
    
    // set steering motor closed loop control gains
    m_LFSteerMotor.config_kP(0, 0.1, 0);
    m_RFSteerMotor.config_kP(0, 0.1, 0);
    m_LRSteerMotor.config_kP(0, 0.1, 0);
    m_RRSteerMotor.config_kP(0, 0.1, 0);
    m_LFSteerMotor.config_kI(0, 0.001, 0);
    m_RFSteerMotor.config_kI(0, 0.001, 0);
    m_LRSteerMotor.config_kI(0, 0.001, 0);
    m_RRSteerMotor.config_kI(0, 0.001, 0);
    //m_LFSteerMotor.config_kD(0, 0.05, 0);
    //m_RFSteerMotor.config_kD(0, 0.05, 0);
    //m_LRSteerMotor.config_kD(0, 0.05, 0);
    //m_RRSteerMotor.config_kD(0, 0.05, 0);

    // accumulate error only if witin 10deg of target - experimental - leave commented out
    //m_LFSteerMotor.config_IntegralZone(0, 5.0*DEG_TO_ENCODERPULSE);
    //m_RFSteerMotor.config_IntegralZone(0, 5*DEG_TO_ENCODERPULSE);
    //m_LRSteerMotor.config_IntegralZone(0, 5*DEG_TO_ENCODERPULSE);
    //m_RRSteerMotor.config_IntegralZone(0, 5*DEG_TO_ENCODERPULSE);

    // allow accumulate up to ~90,000x1ms error  (i.e. 1deg error for 1s)
    // 90,000 x Igain(=0.001) = 90,  causing motor to apply 90/1023 full voltage
    m_LFSteerMotor.configMaxIntegralAccumulator(0, 1250.0*DEG_TO_ENCODERPULSE);
    m_RFSteerMotor.configMaxIntegralAccumulator(0, 1250.0*DEG_TO_ENCODERPULSE);
    m_LRSteerMotor.configMaxIntegralAccumulator(0, 1250.0*DEG_TO_ENCODERPULSE);
    m_RRSteerMotor.configMaxIntegralAccumulator(0, 1250.0*DEG_TO_ENCODERPULSE);
   
    // limit steer ramp rate - experimental only - leave commented out
    //m_LFSteerMotor.configClosedloopRamp(0.5);
    //m_RFSteerMotor.configClosedloopRamp(0.5);
    //m_LRSteerMotor.configClosedloopRamp(0.5);
    //m_RRSteerMotor.configClosedloopRamp(0.5);

    // create drive motors and initalize to factory default
    m_LFDriveMotor = new TalonFX(RobotMap.CANID.LF_DRIVE_MOTOR);
    m_RFDriveMotor = new TalonFX(RobotMap.CANID.RF_DRIVE_MOTOR);
    m_LRDriveMotor = new TalonFX(RobotMap.CANID.LR_DRIVE_MOTOR);
    m_RRDriveMotor = new TalonFX(RobotMap.CANID.RR_DRIVE_MOTOR);
    m_LFDriveMotor.configFactoryDefault();
    m_RFDriveMotor.configFactoryDefault();
    m_LRDriveMotor.configFactoryDefault();
    m_RRDriveMotor.configFactoryDefault();
    m_LFDriveMotor.configNeutralDeadband(0.001);
    m_RFDriveMotor.configNeutralDeadband(0.001);
    m_LRDriveMotor.configNeutralDeadband(0.001);
    m_RRDriveMotor.configNeutralDeadband(0.001);
    m_LFDriveMotor.setNeutralMode(NeutralMode.Coast);
    m_RFDriveMotor.setNeutralMode(NeutralMode.Coast);
    m_LRDriveMotor.setNeutralMode(NeutralMode.Coast);
    m_RRDriveMotor.setNeutralMode(NeutralMode.Coast);

    // set drive motor closed loop control gains
    m_LFDriveMotor.config_kF(0, 0.05, 0);
    m_RFDriveMotor.config_kF(0, 0.05, 0);
    m_LRDriveMotor.config_kF(0, 0.05, 0);
    m_RRDriveMotor.config_kF(0, 0.05, 0);
    m_LFDriveMotor.config_kP(0, 0.1, 0);
    m_RFDriveMotor.config_kP(0, 0.1, 0);
    m_LRDriveMotor.config_kP(0, 0.1, 0);
    m_RRDriveMotor.config_kP(0, 0.1, 0);
    m_LFDriveMotor.config_kI(0, 0.0001, 0);
    m_RFDriveMotor.config_kI(0, 0.0001, 0);
    m_LRDriveMotor.config_kI(0, 0.0001, 0);
    m_RRDriveMotor.config_kI(0, 0.0001, 0);
    
    // limit acculated error to 0.15m of travel  (1/2 of a wheel rotation)
    // at maximum integrated erorr, robot could overtravel by up to 15cm
    m_LFDriveMotor.configMaxIntegralAccumulator(0, 0.15*METERS_TO_ENCODERPULSE);
    m_RFDriveMotor.configMaxIntegralAccumulator(0, 0.15*METERS_TO_ENCODERPULSE);
    m_LRDriveMotor.configMaxIntegralAccumulator(0, 0.15*METERS_TO_ENCODERPULSE);
    m_RRDriveMotor.configMaxIntegralAccumulator(0, 0.15*METERS_TO_ENCODERPULSE);


    // initialize encoders of each steer motor according to CANCoder positions
    ResetSteerEncoders();

    // create subsystem shuffle board page
    initializeShuffleboard();
  }

  
  // seed the encoder value of steering motors based on CANcoder and alignment position
  public void ResetSteerEncoders() {
    m_LFSteerMotor.setSelectedSensorPosition((m_LFCanCoder.getAbsolutePosition()-(3.4)) * DEG_TO_ENCODERPULSE, 0, 0);
    m_RFSteerMotor.setSelectedSensorPosition((m_RFCanCoder.getAbsolutePosition()-(0.97)) * DEG_TO_ENCODERPULSE, 0, 0);
    m_LRSteerMotor.setSelectedSensorPosition((m_LRCanCoder.getAbsolutePosition()-(-2.8)) * DEG_TO_ENCODERPULSE, 0, 0);
    m_RRSteerMotor.setSelectedSensorPosition((m_RRCanCoder.getAbsolutePosition()-(-3.1)) * DEG_TO_ENCODERPULSE, 0, 0);
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

  
  /** Set Robot Chassis Speed - (i.e. drive robot at selcted speeds)
      ChassisSpeeds x,y in m/s, omega in rad/s */
  public void drive(double dx, double dy, double omega, boolean fieldOriented)
    { drive (new ChassisSpeeds(dx, dy, omega), fieldOriented); }

  public void drive(ChassisSpeeds speed, boolean fieldOriented) {
  
    // if chassis speed relative to field, then convert so it is relative to robot
    if (fieldOriented) {
      // convert speeds from field relative according to current gyro angle 
      speed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, Rotation2d.fromDegrees(RobotContainer.gyro.getYaw()));
    }
      
    // determine desired swerve module states from desired chassis speeds
    m_states = m_kinematics.toSwerveModuleStates(speed);
    
    // if desired speed of a swerve(s) exceed maximum possible, then reduce all speeds while maintaining ratio
    SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_METERS_PER_SECOND);

    // assume drive motors driving in forward direction until determined otherwise
    double LFDriveDir = 1.0;
    double RFDriveDir = 1.0;
    double LRDriveDir = 1.0;
    double RRDriveDir = 1.0;

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


    // ---------- Angle Determination for RF Swerve

    double adder2=0.0;
    if (m_RFDriveMotor.getClosedLoopTarget()<0.0)
      { adder2 = 180.0; RFDriveDir = -1.0; }
    double RFCurrentAngleDeg = m_RFSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG;
    double RFAngleDiff = Utils.AngleDifference(RFCurrentAngleDeg%360.0, adder2+m_states[1].angle.getDegrees());
    if (RFAngleDiff<-90.0)
      { RFDriveDir *= -1.0; RFAngleDiff+=180.0; }
    else if (RFAngleDiff>90)
      { RFDriveDir *= -1.0; RFAngleDiff-=180.0; }
    m_RFSteerMotor.set(ControlMode.Position, (RFCurrentAngleDeg + RFAngleDiff)*DEG_TO_ENCODERPULSE);


    // ---------- Angle Determination for LR Swerve

    double adder3=0.0;
    if (m_LRDriveMotor.getClosedLoopTarget()<0.0)
      { adder3 = 180.0; LRDriveDir = -1.0; }
    double LRCurrentAngleDeg = m_LRSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG;
    double LRAngleDiff = Utils.AngleDifference(LRCurrentAngleDeg%360.0, adder3+m_states[2].angle.getDegrees());
    if (LRAngleDiff<-90.0)
      { LRDriveDir *= -1.0; LRAngleDiff+=180.0; }
    else if (LRAngleDiff>90)
      { LRDriveDir *= -1.0; LRAngleDiff-=180.0; }
    m_LRSteerMotor.set(ControlMode.Position, (LRCurrentAngleDeg + LRAngleDiff)*DEG_TO_ENCODERPULSE);


    // ---------- Angle Determination for RR Swerve

    double adder4=0.0;
    if (m_RRDriveMotor.getClosedLoopTarget()<0.0)
      { adder4 = 180.0; RRDriveDir = -1.0; }
    double RRCurrentAngleDeg = m_RRSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG;
    double RRAngleDiff = Utils.AngleDifference(RRCurrentAngleDeg%360.0, adder4+m_states[3].angle.getDegrees());
    if (RRAngleDiff<-90.0)
      { RRDriveDir *= -1.0; RRAngleDiff+=180.0; }
    else if (RRAngleDiff>90)
      { RRDriveDir *= -1.0; RRAngleDiff-=180.0; }
    m_RRSteerMotor.set(ControlMode.Position, (RRCurrentAngleDeg + RRAngleDiff)*DEG_TO_ENCODERPULSE);


    // ---------- Set Drive Motor Speeds

    // go ahead and set motor closed loop target speeds (in encoder pulses per 100ms)
    m_LFDriveMotor.set(ControlMode.Velocity, m_states[0].speedMetersPerSecond*LFDriveDir*METERS_TO_ENCODERPULSE*0.1);
    m_RFDriveMotor.set(ControlMode.Velocity, m_states[1].speedMetersPerSecond*RFDriveDir*METERS_TO_ENCODERPULSE*0.1);
    m_LRDriveMotor.set(ControlMode.Velocity, m_states[2].speedMetersPerSecond*LRDriveDir*METERS_TO_ENCODERPULSE*0.1);
    m_RRDriveMotor.set(ControlMode.Velocity, m_states[3].speedMetersPerSecond*RRDriveDir*METERS_TO_ENCODERPULSE*0.1);
  }


// returns positions of all swerve modules
public SwerveModulePosition[] GetSwerveDistances() {
 
    // create array of module positions to return
    SwerveModulePosition[] states = new SwerveModulePosition[4];
            
    // populate distance(m) and angle for LF swerve
    states[0] = new SwerveModulePosition();
    states[0].distanceMeters = m_LFDriveMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_METERS;
    states[0].angle = Rotation2d.fromDegrees(m_LFSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG);

    // populate distance(m) and angle for RF swerve
    states[1] = new SwerveModulePosition();
    states[1].distanceMeters = m_RFDriveMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_METERS;
    states[1].angle = Rotation2d.fromDegrees(m_RFSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG);

    // populate distance(m) and angle for LR swerve
    states[2] = new SwerveModulePosition();
    states[2].distanceMeters = m_LRDriveMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_METERS;
    states[2].angle = Rotation2d.fromDegrees(m_LRSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG);

    // populate distance(m) and angle for RR swerve
    states[3] = new SwerveModulePosition();
    states[3].distanceMeters = m_RRDriveMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_METERS;
    states[3].angle = Rotation2d.fromDegrees(m_RRSteerMotor.getSelectedSensorPosition() * ENCODERPULSE_TO_DEG);

    return states;
}

/** Returns kinematics of drive system */
public SwerveDriveKinematics getKinematics() {
  return m_kinematics;
}


  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("SwerveDrive");

    // create controls to left-front swerve data
    ShuffleboardLayout l1 = Tab.getLayout("Left-Front", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 5);
    m_LFCanCoderPos = l1.add("CanCoder Deg", 0.0).getEntry();
    m_LFSteerMotorPos = l1.add("Steer Pos'n", 0.0).getEntry();
    m_LFSteerMotorTarget = l1.add("Steer Target", 0.0).getEntry();
    m_LFDriveMotorPos = l1.add("Drive Pos'n", 0.0).getEntry();
    m_LFDriveMotorSpeed = l1.add("Drive Speed", 0.0).getEntry();
    m_LFDriveMotorTargetSpeed = l1.add("Drive Target Spd", 0.0).getEntry();
    m_LFDriveMotorVolts = l1.add("Volts", 0.0).getEntry();
    m_LFDriveMotorTemp = l1.add("Dr Mtr(degC)", 0.0).getEntry();

    // create controls to right-front swerve data
    ShuffleboardLayout l2 = Tab.getLayout("Right-Front", BuiltInLayouts.kList);
    l2.withPosition(1, 0);
    l2.withSize(1, 5);
    m_RFCanCoderPos = l2.add("CanCoder Deg", 0.0).getEntry();
    m_RFSteerMotorPos = l2.add("Steer Posn", 0.0).getEntry();
    m_RFSteerMotorTarget = l2.add("Steer Target", 0.0).getEntry();
    m_RFDriveMotorPos = l2.add("Drive Pos'n", 0.0).getEntry();
    m_RFDriveMotorSpeed = l2.add("Drive Speed", 0.0).getEntry();
    m_RFDriveMotorTargetSpeed = l2.add("Drive Target Spd", 0.0).getEntry();
    m_RFDriveMotorVolts = l2.add("Volts", 0.0).getEntry();
    m_RFDriveMotorTemp = l2.add("Dr Mtr(degC)", 0.0).getEntry();

    // create controls to left-rear swerve data
    ShuffleboardLayout l3 = Tab.getLayout("Left-Rear", BuiltInLayouts.kList);
    l3.withPosition(2, 0);
    l3.withSize(1, 5);
    m_LRCanCoderPos = l3.add("CanCoder Deg", 0.0).getEntry();
    m_LRSteerMotorPos = l3.add("Steer Posn", 0.0).getEntry();
    m_LRSteerMotorTarget = l3.add("Steer Target", 0.0).getEntry();
    m_LRDriveMotorPos = l3.add("Drive Pos'n", 0.0).getEntry();
    m_LRDriveMotorSpeed = l3.add("Drive Speed", 0.0).getEntry();
    m_LRDriveMotorTargetSpeed = l3.add("Drive Target Spd", 0.0).getEntry();
    m_LRDriveMotorVolts = l3.add("Volts", 0.0).getEntry();
    m_LRDriveMotorTemp = l3.add("Dr Mtr(degC)", 0.0).getEntry();

    // create controls to right-rear swerve data
    ShuffleboardLayout l4 = Tab.getLayout("Right-Rear", BuiltInLayouts.kList);
    l4.withPosition(3, 0);
    l4.withSize(1, 5);
    m_RRCanCoderPos = l4.add("CanCoder Deg", 0.0).getEntry();
    m_RRSteerMotorPos = l4.add("Steer Posn", 0.0).getEntry();
    m_RRSteerMotorTarget = l4.add("Steer Target", 0.0).getEntry();
    m_RRDriveMotorPos = l4.add("Drive Pos'n", 0.0).getEntry();
    m_RRDriveMotorSpeed = l4.add("Drive Speed", 0.0).getEntry();
    m_RRDriveMotorTargetSpeed = l4.add("Drive Target Spd", 0.0).getEntry();
    m_RRDriveMotorVolts = l4.add("Volts", 0.0).getEntry();
    m_RRDriveMotorTemp = l4.add("Dr Mtr(degC)", 0.0).getEntry();

    // create controls to bus voltage
    ShuffleboardLayout l5 = Tab.getLayout("Battery", BuiltInLayouts.kList);
    l5.withPosition(4, 0);
    l5.withSize(1, 1);
    m_BattVolts = l5.add("Volts", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    
    // update CANCoder position values (degrees)
    m_LFCanCoderPos.setDouble(m_LFCanCoder.getAbsolutePosition());
    m_RFCanCoderPos.setDouble(m_RFCanCoder.getAbsolutePosition());
    m_LRCanCoderPos.setDouble(m_LRCanCoder.getAbsolutePosition());
    m_RRCanCoderPos.setDouble(m_RRCanCoder.getAbsolutePosition());
    
    // update steer motor position values (encoder pulses)
    m_LFSteerMotorPos.setDouble(m_LFSteerMotor.getSelectedSensorPosition());
    m_RFSteerMotorPos.setDouble(m_RFSteerMotor.getSelectedSensorPosition());
    m_LRSteerMotorPos.setDouble(m_LRSteerMotor.getSelectedSensorPosition());
    m_RRSteerMotorPos.setDouble(m_RRSteerMotor.getSelectedSensorPosition());

    // update steer motor target values (encoder pulses)
    m_LFSteerMotorTarget.setDouble(m_LFSteerMotor.getClosedLoopTarget());
    m_RFSteerMotorTarget.setDouble(m_RFSteerMotor.getClosedLoopTarget());
    m_LRSteerMotorTarget.setDouble(m_LRSteerMotor.getClosedLoopTarget());
    m_RRSteerMotorTarget.setDouble(m_RRSteerMotor.getClosedLoopTarget());

    // update drive motor position values (encoder pulses)
    m_LFDriveMotorPos.setDouble(m_LFDriveMotor.getSelectedSensorPosition());
    m_RFDriveMotorPos.setDouble(m_RFDriveMotor.getSelectedSensorPosition());
    m_LRDriveMotorPos.setDouble(m_LRDriveMotor.getSelectedSensorPosition());
    m_RRDriveMotorPos.setDouble(m_RRDriveMotor.getSelectedSensorPosition());

    // update drive motor speeds (encoder pulses)
    m_LFDriveMotorSpeed.setDouble(m_LFDriveMotor.getSelectedSensorVelocity());
    m_RFDriveMotorSpeed.setDouble(m_RFDriveMotor.getSelectedSensorVelocity());
    m_LRDriveMotorSpeed.setDouble(m_LRDriveMotor.getSelectedSensorVelocity());
    m_RRDriveMotorSpeed.setDouble(m_RRDriveMotor.getSelectedSensorVelocity());

    // update drive motor targets (encoder pulses)
    m_LFDriveMotorTargetSpeed.setDouble(m_LFDriveMotor.getClosedLoopTarget());
    m_RFDriveMotorTargetSpeed.setDouble(m_RFDriveMotor.getClosedLoopTarget());
    m_LRDriveMotorTargetSpeed.setDouble(m_LRDriveMotor.getClosedLoopTarget());
    m_RRDriveMotorTargetSpeed.setDouble(m_RRDriveMotor.getClosedLoopTarget());

    // update drive motor volts
    m_LFDriveMotorVolts.setDouble(m_LFDriveMotor.getMotorOutputVoltage());
    m_RFDriveMotorVolts.setDouble(m_RFDriveMotor.getMotorOutputVoltage());
    m_LRDriveMotorVolts.setDouble(m_LRDriveMotor.getMotorOutputVoltage());
    m_RRDriveMotorVolts.setDouble(m_RRDriveMotor.getMotorOutputVoltage());

    // update drive motor temperatures (degC)
    m_LFDriveMotorTemp.setDouble(m_LFDriveMotor.getTemperature());
    m_RFDriveMotorTemp.setDouble(m_RFDriveMotor.getTemperature());
    m_LRDriveMotorTemp.setDouble(m_LRDriveMotor.getTemperature());
    m_RRDriveMotorTemp.setDouble(m_RRDriveMotor.getTemperature());

    // update battery voltage (volts)
    m_BattVolts.setDouble(m_LFDriveMotor.getBusVoltage());
  }
}
