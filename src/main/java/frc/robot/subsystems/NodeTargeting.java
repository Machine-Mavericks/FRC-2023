// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NodeTargeting extends SubsystemBase {
  Limelight m_nodeTargetCamera;
  int m_numOfRetroTargets;
  double[] Txs;
  double[] Tys;

  /** Creates a new NodeTargeting. */
  public NodeTargeting() {
    m_nodeTargetCamera = RobotContainer.limelight1; // name TBD
    m_nodeTargetCamera.setPipeline(1); // pipeline name TBD
    m_numOfRetroTargets = m_nodeTargetCamera.getNumOfRetro();
  }

  @Override
  public void periodic() {
    double[] Txs = new double[m_nodeTargetCamera.getNumOfRetro()];
    double[] Tys = new double[m_nodeTargetCamera.getNumOfRetro()];
    // This method will be called once per scheduler run
    for (int i = 0; i < m_nodeTargetCamera.getNumOfRetro(); i++){
      Txs[i] = m_nodeTargetCamera.getTargetTx(i);
      Tys[i] = m_nodeTargetCamera.getTargetTx(i);
      this.Txs = Txs;
      this.Tys = Tys;
    }
  }

  public boolean IsTarget() {
    return m_nodeTargetCamera.isTargetPresent()==1;
  }

  public void setLeftBottomTarget(){
    double tempx = this.Txs[0];
    int tempi = 0;
    int templast = 0;
    for (int i = 1; i < m_nodeTargetCamera.getNumOfRetro(); i++){
      if (Txs[i]<tempx){
        templast = tempi;
        tempx = Txs[i];
        tempi = i;
      }
    }
    if (tempi == 0){
      double tempx2 = this.Txs[1];
      for (int i = 1; i < m_nodeTargetCamera.getNumOfRetro(); i++){
        if (Txs[i]<tempx2){
          templast = tempi;
          tempx = Txs[i];
          tempi = i;
        }
      }
    }
    if (this.Tys[templast]<this.Tys[tempi]){
      m_nodeTargetCamera.setRetroTarget(templast);
    } else {
      m_nodeTargetCamera.setRetroTarget(tempi);
    }
  }

  public void setLeftTopTarget(){
    double tempx = this.Txs[0];
    int tempi = 0;
    int templast = 0;
    for (int i = 1; i < m_nodeTargetCamera.getNumOfRetro(); i++){
      if (Txs[i]<tempx){
        templast = tempi;
        tempx = Txs[i];
        tempi = i;
      }
    }
    if (tempi == 0){
      double tempx2 = this.Txs[1];
      for (int i = 1; i < m_nodeTargetCamera.getNumOfRetro(); i++){
        if (Txs[i]<tempx2){
          templast = tempi;
          tempx = Txs[i];
          tempi = i;
        }
      }
    }
    if (this.Tys[templast]>this.Tys[tempi]){
      m_nodeTargetCamera.setRetroTarget(templast);
    } else {
      m_nodeTargetCamera.setRetroTarget(tempi);
    }
  }

  public void setRightBottomTarget(){
    double tempx = this.Txs[0];
    int tempi = 0;
    int templast = 0;
    for (int i = 1; i > m_nodeTargetCamera.getNumOfRetro(); i++){
      if (Txs[i]<tempx){
        templast = tempi;
        tempx = Txs[i];
        tempi = i;
      }
    }
    if (tempi == 0){
      double tempx2 = this.Txs[1];
      for (int i = 1; i > m_nodeTargetCamera.getNumOfRetro(); i++){
        if (Txs[i]<tempx2){
          templast = tempi;
          tempx = Txs[i];
          tempi = i;
        }
      }
    }
    if (this.Tys[templast]<this.Tys[tempi]){
      m_nodeTargetCamera.setRetroTarget(templast);
    } else {
      m_nodeTargetCamera.setRetroTarget(tempi);
    }
  }

  public void setRightTopTarget(){
    double tempx = this.Txs[0];
    int tempi = 0;
    int templast = 0;
    for (int i = 1; i > m_nodeTargetCamera.getNumOfRetro(); i++){
      if (Txs[i]<tempx){
        templast = tempi;
        tempx = Txs[i];
        tempi = i;
      }
    }
    if (tempi == 0){
      double tempx2 = this.Txs[1];
      for (int i = 1; i > m_nodeTargetCamera.getNumOfRetro(); i++){
        if (Txs[i]<tempx2){
          templast = tempi;
          tempx = Txs[i];
          tempi = i;
        }
      }
    }
    if (this.Tys[templast]>this.Tys[tempi]){
      m_nodeTargetCamera.setRetroTarget(templast);
    } else {
      m_nodeTargetCamera.setRetroTarget(tempi);
    }
  }

  /**
   * set target to n 
   * n = 0 --> left bottom target
   * n = 1 --> left top target
   * n = 2 --> right bottom target
   * n = 3 --> right top target
   */
  public void setTarget(int n) {
    if (m_nodeTargetCamera.getNumOfRetro()!=1){
      if (n==0){
        setLeftBottomTarget();
      } else if (n==1){
        setLeftTopTarget();
      } else if (n==2){
        setRightBottomTarget();
      } else {
        setRightTopTarget();
      }
    }
  }
}
