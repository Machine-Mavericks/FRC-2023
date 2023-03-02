// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NodeTargeting extends SubsystemBase {
  Limelight m_nodeTargetCamera; // instance camera
  int m_numOfRetroTargets; // num of visible targets
  double[] Txs; // instance array of target txs
  double[] Tys; // instance array of target tys

  /** Creates a new NodeTargeting. */
  public NodeTargeting() {
    // initializes camera object
    m_nodeTargetCamera = new Limelight("med"); // name TBD
    // sets the pipeline to retroreflective
    m_nodeTargetCamera.setPipeline(1); // pipeline name TBD
    // initializes private variable 
    this.m_numOfRetroTargets = m_nodeTargetCamera.getNumOfRetro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // initializes Txs and Tys
    double[] Txs = new double[m_nodeTargetCamera.getNumOfRetro()];
    double[] Tys = new double[m_nodeTargetCamera.getNumOfRetro()];

    // populates instance arrays
    for (int i = 0; i < m_nodeTargetCamera.getNumOfRetro() && i<4; i++){
      Txs[i] = m_nodeTargetCamera.getTargetTx(i);
      Tys[i] = m_nodeTargetCamera.getTargetTy(i);
      this.Txs = Txs;
      this.Tys = Tys;
    }
  }

  /**
   * returns true if there is a retro/node target
   * @return boolean
   */
  public boolean IsTarget() {
    return m_nodeTargetCamera.isTargetPresent()==1;
  }

  /**
   * chooses the left and bottom most target
   * @return tx of target
   */
  public double txLeftBottomTarget(){
      if (IsTarget()){
        double tempx = this.Txs[0];
      int tempi = 0;
      int templast = 0;
      // looks for left and second leftmost target
      for (int i = 0; i < m_nodeTargetCamera.getNumOfRetro(); i++){
        if (Txs[i]<tempx){
          templast = tempi;
          tempx = Txs[i];
          tempi = i;
        }
      }
      // if second leftmost is never populated, populate using remaining targets
      if (tempi == 0){
        double tempx2 = this.Txs[1];
        for (int i = 1; i < m_nodeTargetCamera.getNumOfRetro(); i++){
          if (Txs[i]<tempx2){
            templast = tempi;
            tempx2 = this.Txs[i];
            tempi = i;
          }
        }
      }
      // return tx of the bottom most of the two left most targets
      if (this.Tys[templast]<this.Tys[tempi]){
       return this.Txs[templast];
      } else {
        return this.Txs[tempi];
     }
    } else {
      return 0.0;
    }
  }

  /**
   * chooses the left and top most target
   * @return tx of target
   */
  public double txLeftTopTarget(){
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
      tempi = 1;
      double tempx2 = this.Txs[1];
      for (int i = 1; i < m_nodeTargetCamera.getNumOfRetro(); i++){
        if (Txs[i]<tempx2){
          templast = tempi;
          tempx2 = Txs[i];
          tempi = i;
        }
      }
    }
    if (this.Tys[templast]>this.Tys[tempi]){
      return this.Txs[templast];
    } else {
      return this.Txs[tempi];
    }
  }

  /**
   * chooses the right and bottom most target
   * @return tx of target
   */
  public double txRightBottomTarget(){
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
          tempx2 = Txs[i];
          tempi = i;
        }
      }
    }
    if (this.Tys[templast]<this.Tys[tempi]){
      return this.Txs[templast];
    } else {
      return this.Txs[tempi];
    }
  }

  /**
   * chooses the right and top most target
   * @return tx of target
   */
  public double txRightTopTarget(){
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
          tempx2 = Txs[i];
          tempi = i;
        }
      }
    }
    if (this.Tys[templast]>this.Tys[tempi]){
      return this.Txs[templast];
    } else {
      return this.Txs[tempi];
    }
  }
}
