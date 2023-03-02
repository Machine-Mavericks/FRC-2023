// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.NodeTargeting;
import java.lang.Math;

public class CenterOnNode extends CommandBase {

  private int node;
  private double tx;
  private NodeTargeting NodeTargeting;
  private SwerveDrive swervedrive;
  /** Creates a new CenterOnNode. 
   * int n = 0 --> center on bottom left
   * int n = 1 --> center on top left
   * int n = 2 --> center on bottom right
   * int n = 3 --> center on top right
  */
  public CenterOnNode(int n) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swervedrive);
    node = 0;
  }

  public void updateTx(){
    if (this.node == 0){
      this.tx = NodeTargeting.txLeftBottomTarget();
    } else if (this.node == 1){
      this.tx = NodeTargeting.txLeftTopTarget();
    } else if (this.node == 2){
      this.tx = NodeTargeting.txRightBottomTarget();
    } else {
      this.tx = NodeTargeting.txRightTopTarget();
    }
  }

  public void driveCompensate() {
    updateTx();
    if (this.tx<0){
      swervedrive.drive(0.0, 0.05, 0.0, false, false);
    } else if (this.tx>0) {
      swervedrive.drive(0.0, -0.05, 0.0, false, false);
    } else {
      swervedrive.drive(0.0, 0.0, 0.0, false, false);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ARE we in init ???????????????????????????????????????????????????????????????????????????????????????");
    swervedrive = RobotContainer.swervedrive;
    NodeTargeting = RobotContainer.NodeTargeting;
    updateTx();
    System.out.println("PAST TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX TX");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("----------------------------------------------------------------------------------------------------");
    driveCompensate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ end");
    swervedrive.drive(0.0, 0.0, 0.0, false, false);
    tx = 0;
    System.out.println("PARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARKPARK");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("*********************************************************************************************** isFin");
    return (Math.abs(tx)<=1.0);
  }
}
