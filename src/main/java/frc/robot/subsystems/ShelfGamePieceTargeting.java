// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Utils.GamePieceData;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;




public class ShelfGamePieceTargeting extends SubsystemBase {
  /** Creates a new GamePeiceTargeting. */
  private double m_limelightOffset_X;
  private double m_limelightOffset_Y;

  private Limelight m_gamepiececamera;

  private GenericEntry m_pose_X;
  private GenericEntry m_pose_Y;

  private GamePieceData m_conePose = new GamePieceData(0, 0);

  public ShelfGamePieceTargeting(double limelightOffset_X, double limelightOffset_Y) {
    RobotContainer.limelight_high.setPipeline(2);;

    System.out.println("Shelf Limelight online??"); // Hopefully

    //m_gamepiececamera.setPipeline(2);

    m_limelightOffset_X = limelightOffset_X;
    m_limelightOffset_Y = limelightOffset_Y;

    initializeShuffleboard();
  }

  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    UpdatePoseEstimation();

    updateShuffleboard(); // Consider reducing update rate?

  }

  public void UpdatePoseEstimation(){ // Replace with other data type?

    double ll_relative_X = Math.sin(Math.toRadians(getTargetHorAngle())) * getGamePieceDistance(); // Replace with distance number
    double ll_relative_Y = Math.pow(getGamePieceDistance(), 2) - Math.pow(ll_relative_X, 2);
    ll_relative_Y = Math.sqrt(ll_relative_Y);

    // TODO: Does not account for rotated limelight, need to add that
    if (isTarget()){
      ll_relative_X += m_limelightOffset_X; 
      ll_relative_Y += m_limelightOffset_Y;
    }
    
    


    //System.out.println("AAH: " + ll_relative_X); // Really unfinished

    m_conePose = new GamePieceData(ll_relative_X, ll_relative_Y);
  }

  // -----------  Generic Target Functions ----- //

  public boolean isTarget(){
    if (RobotContainer.limelight_high.isTargetPresent() == 0) { return false; }
    else { return true; }
  }

  public double getTargetHorAngle() {
    double Angle = RobotContainer.limelight_high.getHorizontalTargetOffsetAngle();
    return Angle;
  }

  public double getGamePieceDistance(){
    return RobotContainer.limelight_high.getGamePieceDistance();
  }
  
  public boolean getGamePieceValid(){
    return RobotContainer.limelight_high.getGamePieceValid();
  }
  /** Returns vertical angle of target (deg)*/
  public double getTargetVertAngle() {
    return RobotContainer.limelight_high.getVerticalTargetOffsetAngle();
  }

  /**  */
  /** Returns 2D pose of cone.
   * About 60% done, does not dectect cubes yet, only sees one cone at a time.
   * All coordinates are in cm and are relative to the robot's centre.
   * @return Data is in a custom structure, use m_X and m_Y to get x & y coordinates
   */
  public GamePieceData getTargetPose(){ //
    return m_conePose;
  }

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create arm page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Shelf GamePeiceTargeting");

    m_pose_X = Tab.add("Estimated X Pose", 0.0).getEntry();
    m_pose_Y = Tab.add("Estimated Y Pose", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current values */
  private void updateShuffleboard() {
    m_pose_X.setDouble(m_conePose.m_X);
    m_pose_Y.setDouble(m_conePose.m_Y);
  }


}
