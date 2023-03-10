// Shuffleboard
//
// Initializes and updates shuffleboard
// This module contains code for making and maintaining main shuffleboard page
// Other pages made by the individual subsystems as req'd

package frc.robot.subsystems;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


    /** Create main shuffleboard page
     * Typically contains autonomous commands and other top-level robot controls*/
    private void initializeMainShuffleboardPage() {

        // Create Main Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");
        m_autonomousPath = new SendableChooser<Integer>();

        // add autonomous commands to page -
        m_autonomousPath.addOption("Co-op Cube Path",0);
        m_autonomousPath.addOption("Left Path",1);
        m_autonomousPath.addOption("Right Path", 2);
        m_autonomousPath.setDefaultOption("Co-op Cube Path", 0);

        tab.add("Preround Paths", m_autonomousPath).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);
        m_delayTime = tab.add("Auto Delay Time", 0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1).withSize(1, 1).withProperties(Map.of("min", 0, "max", 10)).getEntry();

              
        // add match time remaining in autonomous/teleop part of match (seconds)
        ShuffleboardLayout l1 = tab.getLayout("Timer", BuiltInLayouts.kList);
        l1.withPosition(0, 2);
        l1.withSize(1, 2);
        m_timeLeft = l1.add("TimeLeft", 0.0).getEntry();
    }

    // returns position of autonomous commands on shuffleboard
    // typically called by Robot AutonomousInit to select auto path to be followed
    public int GetSelectedPath()
    {
        return m_selectedPath;
    }
    



} // end class ShuffleboardOI
