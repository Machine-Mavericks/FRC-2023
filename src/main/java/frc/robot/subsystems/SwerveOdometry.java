// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Swerve odometry is used to estimate current robot x, y position and angle.
// x and y coordinates are relative to when odometry was last reset

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveOdometry extends SubsystemBase {

  // constant to convert degrees to radians
  final float DEGtoRAD = (float) (3.1415926 / 180.0);

  // create swerve drive odometry object
  private SwerveDriveOdometry m_odometry;

  // subsystem shuffleboard controls
  private GenericEntry m_robotX;
  private GenericEntry m_robotY;
  private GenericEntry m_robotAngle;

  /** Creates a new SwerveOdometry. */
  public SwerveOdometry() {

    // create robot odometry - set to (0,0,0)(x,y,ang)

    // initialize swerve drive odometry
    m_odometry = new SwerveDriveOdometry(RobotContainer.swervedrive.getKinematics(),
    new Rotation2d(0.0),
    RobotContainer.swervedrive.GetSwerveDistances() );

    // reset initial position
    setPosition(0.0,0.0,0.0,0.0);

    // create odometry shuffleboard page
    initializeShuffleboard();
  }


  // -------------------- Initialize and Update Odometry Methods


  /** Initialize robot odometry to zero */
  public void InitializetoZero() {
    setPosition(0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Used to set or reset odometry to fixed position
   * x, y displacement in m, robot angle in deg, gyro in deg
   */
  public void setPosition(double x, double y, double robotangle, double gyroangle) {

    // make robot position vector
    Pose2d position = new Pose2d(x, y, new Rotation2d(robotangle * DEGtoRAD));

    // set robot odometry
    m_odometry.resetPosition(new Rotation2d(gyroangle * DEGtoRAD),
    RobotContainer.swervedrive.GetSwerveDistances(),
    position);
  }

  /** Update current robot dometry - called by scheduler at 50Hz */
  @Override
  public void periodic() {

    // get gyro angle (in degrees) and make rotation vector
    Rotation2d gyroangle = new Rotation2d(RobotContainer.gyro2.getYaw() * DEGtoRAD);

    // update odometry
    m_odometry.update(gyroangle, RobotContainer.swervedrive.GetSwerveDistances());
    
    
    // update odemetry shuffleboard page
    updateShuffleboard();
  }

  // -------------------- Robot Current Odometry Access Methods --------------------

  /** return robot's current position vector Pose2d */
  public Pose2d getPose2d() {
    return m_odometry.getPoseMeters();
  }


  // ----------------- FUnctions to record/recall Pos2d

  Pose2d m_MemPoints[] = {new Pose2d(0,0,new Rotation2d(0.0)),
                            new Pose2d(0,0,new Rotation2d(0.0)),
                            new Pose2d(0,0,new Rotation2d(0.0)) };

  /** saves Pose2D coordinate for later recall
   * num = 0 to 2 (three memories available) */
  public void RecordPose2d(Pose2d point, int num)
  {
      if (num<m_MemPoints.length)
        m_MemPoints[num] = point;
  }

  /** recalls Pose2D coordinate previously saved 
   * num = 0 to 2 (three memories available) */
  public Pose2d RecallPoint(int num)
  {
    // return saved point.  If not in range, simply return 0,0,0 point
    if (num<m_MemPoints.length)
      return m_MemPoints[num];
    else
      return new Pose2d(0,0,new Rotation2d(0.0));
  }



  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Odometry");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Odometry", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_robotX = l1.add("X (m)", 0.0).getEntry();
    m_robotY = l1.add("Y (m)", 0.0).getEntry();
    m_robotAngle = l1.add("Angle(deg)", 0.0).getEntry();

  }

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    // write current robot odometry
    m_robotX.setDouble(getPose2d().getX());
    m_robotY.setDouble(getPose2d().getY());
    m_robotAngle.setDouble(getPose2d().getRotation().getDegrees());
  }

} // end SwerveOdometry Class