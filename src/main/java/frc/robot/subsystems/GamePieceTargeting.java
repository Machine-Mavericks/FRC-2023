// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceTargeting extends SubsystemBase {
  /** Creates a new GamePeiceTargeting. */
  private Limelight m_gamepiececamera;
  public GamePieceTargeting() {
    m_gamepiececamera = new Limelight("limelight-test");

    System.out.println("Limelight online??"); // Hopefully

    m_gamepiececamera.setPipeline(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // -----------  Generic Target Functions ----- //

  public boolean isTarget(){
    if (m_gamepiececamera.isTargetPresent() == 0) { return false; }
    else { return true; }
  }

  public double getTargetHorAngle() {
    double Angle = m_gamepiececamera.getHorizontalTargetOffsetAngle();
    return Angle;
  }
  
  /** Returns vertical angle of target (deg)*/
  public double getTargetVertAngle() {
    return m_gamepiececamera.getVerticalTargetOffsetAngle();
  }
}
