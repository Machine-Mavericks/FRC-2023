// Shuffleboard
//
// Initializes and updates shuffleboard
// This module contains code for making and maintaining main shuffleboard page
// Other pages made by the individual subsystems as req'd

package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;


/** Contains shuffleboard setup for generic main page not belonging to any subsubsystem
 * Typically used to have robot-level settings and command functions */
public class AutoPathSelect extends SubsystemBase {

    // example autonomous path shuffleboard selection boxes
    // true if selected, false if not
    // <add any other controls here that go on main shufflebard page
    private GenericEntry m_delayTime;
    private SendableChooser<Integer> m_autonomousPath;

    // other controls on main page
    private GenericEntry m_timeLeft;
    private Integer m_selectedPath;

    /** Initializes the Shuffleboard
     * Creates the subsystem pages */
    public AutoPathSelect() {
        // add autonomous commands to shuffleboard
        initializeMainShuffleboardPage();
    }

    /** Update Shuffleboard Pages. This method will be called once per scheduler run
     * (=50Hz) */
    @Override
    public void periodic() {

        // update main page
        // update remaining time in match (rounded to nearest second)
        m_selectedPath = (Integer)m_autonomousPath.getSelected();
        m_timeLeft.setDouble(Math.round(Timer.getMatchTime()));
    }

    
    /** returns delay for autonomous routines */
    public double getAutoDelay()
    {
        return m_delayTime.getDouble(0.0);
    }

    // -------------------- Shuffboard Methods --------------------

    private static String fileToString(File file){
        byte[] fileBytes = new byte[0];
        String fileContents;
        // Use the readAllBytes method of the Files class to read the contents of the file into a byte array 
        try{
          fileBytes = Files.readAllBytes(file.toPath());   
          fileContents = new String(fileBytes, StandardCharsets.UTF_8);    
        }
        catch(IOException e){
          e.printStackTrace();
          fileContents = "Non existent";
        }
        return fileContents;
      }

    /** Create main shuffleboard page
     * Typically contains autonomous commands and other top-level robot controls*/
    private void initializeMainShuffleboardPage() {

        File deployDir = Filesystem.getDeployDirectory(); 
        File branchFile = new File(deployDir, "branch.txt"); 
        File commitFile = new File(deployDir, "commit.txt");

        // Create Main Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");
        m_autonomousPath = new SendableChooser<Integer>();

        // add autonomous commands to page -
        m_autonomousPath.addOption("Co-op Cube Path",0);
        m_autonomousPath.addOption("Left Path",1);
        m_autonomousPath.addOption("Right Path", 2);
        m_autonomousPath.setDefaultOption("Co-op Cube Path", 0);

        tab.add("Preround Paths", m_autonomousPath).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);
        m_delayTime = tab.add("Auto Delay Time", 0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1).withSize(1, 1).withProperties(Map.of("min", 0, "max", 15)).getEntry();

              
        // add match time remaining in autonomous/teleop part of match (seconds)
        ShuffleboardLayout l1 = tab.getLayout("Timer", BuiltInLayouts.kList);
        l1.withPosition(0, 2);
        l1.withSize(1, 2);
        m_timeLeft = l1.add("TimeLeft", 0.0).getEntry();

        ShuffleboardLayout l2 = tab.getLayout("Software version", BuiltInLayouts.kList);
        l2.withPosition(6, 0);
        l2.withSize(3, 2);
        GenericEntry m_branch = l2.add("Branch", "None").getEntry();
        GenericEntry m_commit = l2.add("Commit", "None").getEntry();;

        m_branch.setString(fileToString(branchFile));
        m_commit.setString(fileToString(commitFile));
        }

    // returns position of autonomous commands on shuffleboard
    // typically called by Robot AutonomousInit to select auto path to be followed
    public int GetSelectedPath()
    {
        return m_selectedPath;
    }
    



} // end class ShuffleboardOI
