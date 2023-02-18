// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new limelight. */

    // network table to communicate to camera with
    private NetworkTable m_table;

    // subsystem shuffleboard controls
    private GenericEntry m_Pipeline;
    private GenericEntry m_TargetPresent;

    private GenericEntry m_AprilTagID;

    private GenericEntry m_AngleX;
    private GenericEntry m_AngleY;
    private GenericEntry m_Skew;
    private GenericEntry m_Area;
    private GenericEntry m_Short;
    private GenericEntry m_Long;
    private GenericEntry m_Hor;
    private GenericEntry m_Vert;
    private GenericEntry m_X;
    private GenericEntry m_Y;
    private GenericEntry m_Z;
    private GenericEntry m_Pitch;
    private GenericEntry m_Yaw;
    private GenericEntry m_Roll;

    private GenericEntry m_Xfs;
    private GenericEntry m_Yfs;
    private GenericEntry m_Zfs;
    private GenericEntry m_Pitchfs;
    private GenericEntry m_Yawfs;
    private GenericEntry m_Rollfs;
    private GenericEntry m_Xrs;
    private GenericEntry m_Yrs;
    private GenericEntry m_Zrs;
    private GenericEntry m_Pitchrs;
    private GenericEntry m_Yawrs;
    private GenericEntry m_Rollrs;

  
    /**
     * Creates a new Limelight.
     * Input: String containing name of limelight (defined in the camera)
     */
    public Limelight(String name) {
  
      // set pointer to limelight network table
      m_table = NetworkTableInstance.getDefault().getTable("limelight-"+name);

      // initialize camera to use LED mode set in the current pipeline setup
      m_table.getEntry("ledMode").setNumber(0);

      // set camera streaming mode - primary and secondary cameras are placed
      // side-by-side
      m_table.getEntry("stream").setNumber(0);


      // set initial pipeline to 0
      setPipeline(0);
  
      // create shuffleboard page
      initializeShuffleboard(name);
    }
  
    int m_UpdateTimer = 0;
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      updateShuffleboard();
      // update shuffleboard - update at 5Hz is sufficient for this subsystem
      //m_UpdateTimer++;
      //if (m_UpdateTimer>=10)
      //{
        updateShuffleboard();
      //  m_UpdateTimer=0;
      //}
    }
  
    // ---------- Camera Control Functions ----------
  
    /** set camera's current pipeline: 0 to 9 */

    public void setPipeline(int num) {

      if (num >= 0 && num <= 9)
      m_table.getEntry("pipeline").setNumber(num);
    }
  
    /** returns camera's current pipeline: 0 to 9 */

    public Double getPipeline() {

      return m_table.getEntry("getPipe").getDouble(0);
    }
  
    // ---------------Camera Access Functions ---------------------
  
    /**
     * get horitaonal angle from center of camera view to center of target
     * returns -27 to +27 degrees
     */
    public float getHorizontalTargetOffsetAngle() {
      return m_table.getEntry("tx").getFloat(0);
    }
  
    /**
     * get vertical angle from center of camera view to center of target
     * returns -20 to +20 degrees
     */
     float getVerticalTargetOffsetAngle() {
      
      return m_table.getEntry("ty").getFloat(0);
    }
  
    /** get rotation angle between view of camera of target */
    float getTargetSkew() {
      return m_table.getEntry("ts").getFloat(0);
    }
  
    /** Camera translation vector definition */
    public class CamTran {
      public double x;
      public double y;
      public double z;
      public double pitch;
      public double yaw;
      public double roll;
  
      public CamTran() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        roll = 0.0;
      }
    };

    /** get camera translation vector to target */
    CamTran getCameraTranslation() {
  
      // set up data structure to return
      CamTran camtran = new CamTran();
  
      // get camera translation vector from camera
      double[] vector = m_table.getEntry("camTran").getDoubleArray(new double[]{});
     
      // if translation vector is valid (has 6 numbers in it) go ahead and record data in structure
      if (vector.length>=6)
        {
        camtran.x = vector[0];
        camtran.y = vector[1];
        camtran.z = vector[2];
        camtran.pitch = vector[3];
        camtran.yaw = vector[4];
        camtran.roll = vector[5];
      } 
  
      // return data structure
      return camtran;
    }

    /** Robot Pose in field space as computed by this fiducial (x,y,z,rx,ry,rz) definition */
      public class t6r_fs {
        public double xfs;
        public double yfs;
        public double zfs;
        public double pitchfs;
        public double yawfs;
        public double rollfs;
      
        public t6r_fs() {
          xfs = 0.0;
          yfs = 0.0;
          zfs = 0.0;
          pitchfs = 0.0;
          yawfs = 0.0;
          rollfs = 0.0;
        }
      };
      
      /** get Robot pose (field space) */
      t6r_fs gett6r_fs() {
    
      // set up data structure to return
      t6r_fs t6r_fs = new t6r_fs();
      
      // get robot pose in field space vector from camera
      double[] vectorfs = m_table.getEntry("botpose").getDoubleArray(new double[]{});
         
      // if translation vector is valid (has 6 numbers in it) go ahead and record data in structure
      if (vectorfs.length>=6)
      {
        t6r_fs.xfs = vectorfs[0];
        t6r_fs.yfs = vectorfs[1];
        t6r_fs.zfs = vectorfs[2];
        t6r_fs.pitchfs = vectorfs[3];
        t6r_fs.yawfs = vectorfs[4];
        t6r_fs.rollfs = vectorfs[5];
      } 
      
      // return data structure
      return t6r_fs;
      }
  
      /** Target Pose in robot space as computed by this fiducial (x,y,z,rx,ry,rz) definition */
      public class t6t_rs {
        public double xrs;
        public double yrs;
        public double zrs;
        public double pitchrs;
        public double yawrs;
        public double rollrs;

        public t6t_rs() {
          xrs = 0.0;
          yrs = 0.0;
          zrs = 0.0;
          pitchrs = 0.0;
          yawrs = 0.0;
          rollrs = 0.0;
        }
      };

      /** get Target pose (robot space) */
      t6t_rs gett6t_rs() {

        // set up data structure to return
        t6t_rs t6t_rs = new t6t_rs();

        // get target pose in robot space from camera
        double[] vectorrs = m_table.getEntry("botpose_targetspace").getDoubleArray(new double[]{});
 
        // if translation vector is valid (has 6 numbers in it) go ahead and record data in structure
        if (vectorrs.length>=6)
        {
          t6t_rs.xrs = vectorrs[0];
          t6t_rs.yrs = vectorrs[1];
          t6t_rs.zrs = vectorrs[2];
          t6t_rs.pitchrs = vectorrs[3];
          t6t_rs.yawrs = vectorrs[4];
          t6t_rs.rollrs = vectorrs[5];
        } 

        // return data structure
        return t6t_rs;
      }



    // get target detection time latency
    public double getLatencyContribution() {
      return m_table.getEntry("tl").getDouble(0);
    }
  
    /** get whether target is currently detected or not, returns 0 or 1 */ 
    public double isTargetPresent() {
      return m_table.getEntry("tv").getDouble(0);
    }
  
    /** get target area atributes - 0 to 100% of image */
    public double getTargetArea() {
      return m_table.getEntry("ta").getDouble(0);
    }
  
    /** returns shortest sidelength of target fitted bounding box (# pixels) */
    public double getShortestSide() {
      return m_table.getEntry("tshort").getDouble(0);
    }
  
    /** returns longest sidelength of target fitted bounding box (# pixels) */
    public double getLongestSide() {
      return m_table.getEntry("tlong").getDouble(0);
    }
  
    /** Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
    public double getHorizontalSideLength() {
      return m_table.getEntry("thor").getDouble(0);
    }
  
    /** Vertical sidelength of the rough bounding box (0 - 320 pixels) */
    public double getVerticalSideLength() {
      return m_table.getEntry("tvert").getDouble(0);
    }
  
    /** Get primary april tag id */
    public double getPrimAprilTagID () {
      return m_table.getEntry("tid").getDouble(0);
    }

    // ---------- get raw target attributes ----------
  
    /** Raw Screenspace X */
    float getRawScreenspaceX0()
    {  return m_table.getEntry("tx0").getFloat(0); }
  
    /** Raw Screenspace X */
    float getRawScreenspaceX1()
    {  return m_table.getEntry("tx1").getFloat(0); }
  
    /** Raw Screenspace X */
    float getRawScreenspaceX2()
    {  return m_table.getEntry("tx2").getFloat(0); }
  
    /** Raw Screenspace Y */
    float getRawScreenspaceY0()
    {  return m_table.getEntry("ty0").getFloat(0);}
  
    /** Raw Screenspace Y */
    float getRawScreenspaceY1()
    {  return m_table.getEntry("ty1").getFloat(0); }
  
    /** Raw Screenspace Y */
    float getRawScreenspaceY2()
    {  return m_table.getEntry("ty2").getFloat(0); }
  
    /** Area (0% of image to 100% of image) */
    float getRawArea0()
    {  return m_table.getEntry("ta0").getFloat(0);}
  
    /** Area (0% of image to 100% of image) */
    float getRawArea1()
    {  return m_table.getEntry("ta1").getFloat(0);}
  
    /** Area (0% of image to 100% of image) */
    float getRawArea2()
    {  return m_table.getEntry("ta2").getFloat(0);}
  
    /** Skew or rotation (-90 degrees to 0 degrees) */
    float getRawSkew0()
    {  return m_table.getEntry("ts0").getFloat(0);}
  
    /** Skew or rotation (-90 degrees to 0 degrees) */
    float getRawSkew1()
    {  return m_table.getEntry("ts1").getFloat(0);}
  
    /** Skew or rotation (-90 degrees to 0 degrees) */
    float getRawSkew2()
    { return m_table.getEntry("ts2").getFloat(0);}
  
  // -------------------- Subsystem Shuffleboard Methods --------------------


  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard(String name) {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Limelight: "+name);

    // camera pipeline number
    m_Pipeline = Tab.add("Pipeline", 0).withPosition(0,0).getEntry();

    // does camera detect target
    m_TargetPresent = Tab.add("Target Present", false).withPosition(1,0).getEntry();

    // april tag target id
    m_AprilTagID = Tab.add("AprilTag Target ID", 0).withPosition(2,0).getEntry();

    // camera target information
    ShuffleboardLayout l1 = Tab.getLayout("Target", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(1, 4);
    m_AngleX = l1.add("AngleX", 0.0).getEntry();
    m_AngleY = l1.add("AngleY", 0.0).getEntry();
    m_Skew = l1.add("Skew", 0.0).getEntry(); 
    
    // target dimensions
    ShuffleboardLayout l2 = Tab.getLayout("Dimensions", BuiltInLayouts.kList);
    l2.withPosition(3,0);
    l2.withSize(1,5);
    m_Area = l2.add("Area", 0.0).getEntry(); 
    m_Short = l2.add("Short", 0.0).getEntry(); 
    m_Long = l2.add("Long", 0.0).getEntry(); 
    m_Hor = l2.add("Hor", 0.0).getEntry(); 
    m_Vert = l2.add("Vert", 0.0).getEntry();

    // camera translation vector to target
    ShuffleboardLayout l3 = Tab.getLayout("CamTran", BuiltInLayouts.kList);
    l3.withPosition(4,0);
    l3.withSize(1,5);
    m_X = l3.add("X", 0.0).getEntry();
    m_Y = l3.add("Y", 0.0).getEntry(); 
    m_Z = l3.add("Z", 0.0).getEntry(); 
    m_Pitch = l3.add("Pitch", 0.0).getEntry(); 
    m_Yaw = l3.add("Yaw", 0.0).getEntry(); 
    m_Roll = l3.add("Roll", 0.0).getEntry();

    ShuffleboardLayout l4 = Tab.getLayout("Robot in field space", BuiltInLayouts.kList);
    l4.withPosition(6,0);
    l4.withSize(1,5);
    m_Xfs = l4.add("X", 0.0).getEntry();
    m_Yfs = l4.add("Y", 0.0).getEntry(); 
    m_Zfs = l4.add("Z", 0.0).getEntry(); 
    m_Pitchfs = l4.add("Pitch", 0.0).getEntry(); 
    m_Yawfs = l4.add("Yaw", 0.0).getEntry(); 
    m_Rollfs = l4.add("Roll", 0.0).getEntry();

    ShuffleboardLayout l5 = Tab.getLayout("Target in robot space", BuiltInLayouts.kList);
    l5.withPosition(6,0);
    l5.withSize(1,5);
    m_Xrs = l5.add("X", 0.0).getEntry();
    m_Yrs = l5.add("Y", 0.0).getEntry(); 
    m_Zrs = l5.add("Z", 0.0).getEntry(); 
    m_Pitchrs = l5.add("Pitch", 0.0).getEntry(); 
    m_Yawrs = l5.add("Yaw", 0.0).getEntry(); 
    m_Rollrs = l5.add("Roll", 0.0).getEntry();

  }


  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    
    // update camera pipeline and target detected indicator
    m_Pipeline.setDouble(getPipeline());

    m_TargetPresent.setBoolean(isTargetPresent()==1);
    m_AprilTagID.setDouble(getPrimAprilTagID());



    
    // update angles to center of target
    m_AngleX.setDouble(getHorizontalTargetOffsetAngle());
    m_AngleY.setDouble(getVerticalTargetOffsetAngle());
    m_Skew.setDouble(getTargetSkew());

    // update dimensions of target
    m_Area.setDouble(getTargetArea());
    m_Short.setDouble(getShortestSide());
    m_Long.setDouble(getLongestSide());
    m_Hor.setDouble(getHorizontalSideLength());
    m_Vert.setDouble(getVerticalSideLength());

    // update camera translation vector
    CamTran vector = getCameraTranslation();
    m_X.setDouble(vector.x);
    m_Y.setDouble(vector.y);
    m_Z.setDouble(vector.z);
    m_Pitch.setDouble(vector.pitch);
    m_Yaw.setDouble(vector.yaw);
    m_Roll.setDouble(vector.roll);


    // update robot in field space vector
    t6r_fs vectorfs = gett6r_fs();
    m_Xfs.setDouble(vectorfs.xfs);
    m_Yfs.setDouble(vectorfs.yfs);
    m_Zfs.setDouble(vectorfs.zfs);
    m_Pitchfs.setDouble(vectorfs.pitchfs);
    m_Yawfs.setDouble(vectorfs.yawfs);
    m_Rollfs.setDouble(vectorfs.rollfs);

    // update target in robot space vector
    t6t_rs vectorrs = gett6t_rs();
    m_Xrs.setDouble(vectorrs.xrs);
    m_Yrs.setDouble(vectorrs.yrs);
    m_Zrs.setDouble(vectorrs.zrs);
    m_Pitchrs.setDouble(vectorrs.pitchrs);
    m_Yawrs.setDouble(vectorrs.yawrs);
    m_Rollrs.setDouble(vectorrs.rollrs);
  }

} // end class LImelight

