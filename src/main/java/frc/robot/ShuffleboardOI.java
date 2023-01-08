// Shuffleboard
//
// Initializes and updates shuffleboard
// This module contains code for making and maintaining main shuffleboard page
// Other pages made by the individual subsystems as req'd

package frc.robot;


import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.NetworkTableEntry;


/** Contains shuffleboard setup for generic main page not belonging to any subsubsystem
 * Typically used to have robot-level settings and command functions */
public class ShuffleboardOI extends SubsystemBase {

    // example autonomous path shuffleboard selection boxes
    // true if selected, false if not
    // <add any other controls here that go on main shufflebard page
    private NetworkTableEntry m_delayTime;
    private SendableChooser<Integer> m_autonomousPath;

    // other controls on main page
    private NetworkTableEntry m_timeLeft;
    public Integer m_selectedPath;

    /** Initializes the Shuffleboard
     * Creates the subsystem pages */
    public ShuffleboardOI() {

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
       
    }



} // end class ShuffleboardOI
