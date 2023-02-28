// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class TargetSelect extends SubsystemBase {
  
  private GenericEntry m_Indicators[][] = new GenericEntry[9][3];
  private GenericEntry m_TargetPose;
  private GenericEntry m_SelectedIndicator;
  private GenericEntry m_ConeSelect; 
  private GenericEntry m_PickupLeftRightSelect;
  private GenericEntry m_PickupPose;

  // currently selected target position
  private boolean m_TargetSelected;
  private int m_SelectedTargetPosX;
  private int m_SelectedTargetPosY;


  /** Creates a new ShuffleboardTargetSelect. */
  public TargetSelect() {

    // update shuffleboard
    initializeShuffleboard();

    // initially start with no target selected
    ClearCurrentTarget();
  }

  
  // subsystem periodic function - operates at 50Hz
  private int timer = 0;
  @Override
  public void periodic() {
    
    timer++;
    if (timer>=5)
    {
      timer=0;
      // update shuffleboard - find new selected targets
      updateShuffleboard();
    }
    
  }


  // ------------------ Public Funcitons to Get Dropoff Target Info

  // returns true if target is curently selected
  public boolean IsTargetSelected()
  {  return m_TargetSelected; }


  // returns if cone mode is currently selected
  public boolean isConeSelected()
  { return m_ConeSelect.getBoolean(true); }

  // function to return selected target position (indices in 9x3 matrix)
  // returns (-1,-1) if none selected
  class TargetMatrixPos
  {
    public int x, y;
    TargetMatrixPos() {x=-1;y=-1;}
  }
  public TargetMatrixPos GetTargetMatrixPos()
  {
    TargetMatrixPos m = new TargetMatrixPos();
    m.x = m_SelectedTargetPosX;  m.y = m_SelectedTargetPosY; 
    return m;
  }

  // function to return selected target position (in meters)
  // returns position of target in current team (red/blue) coordinates
  // coordinate 0,0 at right-side corner at end with driver stations
  public Pose2d GetTargetPose2d()
  { 
    // temporary 1.0, 1.5 and 2.0 adders are temporary - to be replaced by robot arm length
    // preferrably using constant 
    
    // defautl x and y to use if no target is selected
    double x = 1.19253+1.0; double y = 4.0;

    // if a target is selected
    if (m_TargetSelected)
    {
      // x coordinate is independent of which team we are on
      // apriltags are 40.45" from edge of wall
      switch (m_SelectedTargetPosY) {
        case 2:
          x = 0.03595+2.0; break;    // 14.15" from wall + distance allowance for robot arm
        case 1: 
          x = 0.78613+1.5; break;    // 30.95" from wall + distance allowance for robot arm
        case 0:
          x = 1.19253+1.0; break;    // 46.95" from wall + distance allowance for robot arm
      }
      
      // determine y coordinate
      if (DriverStation.getAlliance() == Alliance.Blue)
      {
        // determine y coordinate for blue team
        y = 0.512826 + (double)m_SelectedTargetPosX * 0.5588;
      }
      else
      {
        // determine y coordinate for red team
        // 315.5" - 42.19" + 22" =  295.31"
        // 295.31" = 7.500874m
        y = 7.500874 - (double)m_SelectedTargetPosX * 0.5588;
      }

    }
  
  // return Pose2d target position
  return new Pose2d(x, y, Rotation2d.fromDegrees(0.0));
  }


  // ------------------ Public Funcitons to Get Pickup Target Info

  // function to return selected target position (in meters)
  // returns position of target in current team (red/blue) coordinates
  // coordinate 0,0 at right-side corner at end with driver stations
  public Pose2d GetPickupTargetPose2d()
  {
    // field length is 54'3.25"  (=16.54175m)
    // field width is 26'3.5" (=8.0137)
    
    // x coordinate same for each team - apriltag xcoordinate - 1.5m distance for robot length (TBD)
    double x=16.1787 - 1.5;
    
    // are we on blue or red team
    double y = 0;
    if (DriverStation.getAlliance() == Alliance.Blue)
    {
      // on blue team. Choose 0.5m to right of left of apriltag y coordinate
      if (m_PickupLeftRightSelect.getBoolean(true))
        y = 6.74 +0.5;
      else
        y = 6.74 - 0.5;
    }
    else
    {
      // on red team. Choose 0.5m to right or left of apriltag y coordinate
      if (m_PickupLeftRightSelect.getBoolean(true))
        y = 8.0137 - 6.74 - 0.5;
      else
        y = 8.0137 - 6.74 + 0.5;
    }
    
    return new Pose2d(x, y, Rotation2d.fromDegrees(180.0));
  }



  // -------------------- Subsystem Shuffleboard Methods --------------------


  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Target Select");

    // add buttons - add cube glyph to cube spots
    for (int y=0;y<=2;++y)
        for (int x=0;x<=8;++x)
        {
          int spacer=0; if (x>2) spacer+=1; if (x>5) spacer+=1;
          
          m_Indicators[x][y] = Tab.add(String.valueOf(x)+","+String.valueOf(y), false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(x+spacer, y)
            .withSize(1,1)
            .getEntry();
        
          // ensure indicator is initially OFF  
          m_Indicators[x][y].setBoolean(false);
          }

    // selected indicator
    m_SelectedIndicator = Tab.add("Target", "")
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(0, 3)
              .withSize(1,1)
              .getEntry();
    // current target position coordinates 
    m_TargetPose = Tab.add("Target Pose", "")
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(1, 3)
              .withSize(1,1)
              .getEntry();

    // cone or cube selection
    m_ConeSelect = Tab.add("Cone Select", true)
              .withWidget(BuiltInWidgets.kToggleSwitch)
              .withPosition(5, 3)
              .withSize(1,1)
              .getEntry(); 

    // pickup target select
    m_PickupLeftRightSelect = Tab.add("L-R", true)
              .withWidget(BuiltInWidgets.kToggleSwitch)
              .withPosition(9, 3)
              .withSize(1,1)
              .getEntry(); 

    // current target position coordinates 
    m_PickupPose = Tab.add("Pickup Pose", "")
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(9, 4)
              .withSize(1,1)
              .getEntry();
    }



  // update shuffleboard
  private void updateShuffleboard() {
    
    // look for any new buttons pressed - to select current target
    boolean found=false;
    for (int y=0;y<=2;++y)
      for (int x=0;x<=8;++x)
      {
        // is button pressed?
        if (m_Indicators[x][y].getBoolean(false)==true)
        {
          // is this not our current target, then make it our new target, and clear last selected target
          if (x!=m_SelectedTargetPosX || y!=m_SelectedTargetPosY)
          { ClearCurrentTarget(); found=true;
            m_SelectedTargetPosX = x; m_SelectedTargetPosY = y; m_TargetSelected = true;}
        }
      }

    // did we just clear current button without pressing any other button?
    if (m_TargetSelected == true && found==false && m_Indicators[m_SelectedTargetPosX][m_SelectedTargetPosY].getBoolean(false)==false)
      ClearCurrentTarget();
    
    // identify selected target and print coordinates on screen
    if (IsTargetSelected())
    {
      m_SelectedIndicator.setString(String.valueOf(m_SelectedTargetPosX)+","+String.valueOf(m_SelectedTargetPosY));
      Pose2d pose = GetTargetPose2d();
      m_TargetPose.setString(String.format("%.2f", pose.getX())+","+String.format("%.2f", pose.getY()));
    
      // if cone position currently selected, then turn on cone switch
      if (m_SelectedTargetPosY==1 || m_SelectedTargetPosY==2)
        if (m_SelectedTargetPosX == 0 || m_SelectedTargetPosX == 2 ||
            m_SelectedTargetPosX == 3 || m_SelectedTargetPosX == 5 ||
            m_SelectedTargetPosX == 6 || m_SelectedTargetPosX == 8)
            
            m_ConeSelect.setBoolean(true);

      // if cube position currently selected, then turn off cone switch
       if (m_SelectedTargetPosY==1 || m_SelectedTargetPosY==2)
        if (m_SelectedTargetPosX == 1 || m_SelectedTargetPosX == 4 ||
            m_SelectedTargetPosX == 7 )

            m_ConeSelect.setBoolean(false);
    
    }
    else
    {
      // no current target. do not show coordinate or position coordinate
      m_SelectedIndicator.setString("");
      m_TargetPose.setString("");
    } 
  
    // update pickup target position on shuffleboard
    Pose2d pose = GetPickupTargetPose2d();
    m_PickupPose.setString(String.format("%.2f", pose.getX())+","+String.format("%.2f", pose.getY()));

  }


  // clears target if one is currently selected
  private void ClearCurrentTarget()
  {
    if (m_TargetSelected)
    { 
      // turn off targeted button
      if (m_SelectedTargetPosX>=0 && m_SelectedTargetPosX<=8 &&
      m_SelectedTargetPosY>=0 && m_SelectedTargetPosY<=2)
        m_Indicators[m_SelectedTargetPosX][m_SelectedTargetPosY].setBoolean(false);
    }
    
    m_TargetSelected=false;
    m_SelectedTargetPosX=-1; m_SelectedTargetPosY=-1;
  }

}
