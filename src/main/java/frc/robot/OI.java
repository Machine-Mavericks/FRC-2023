package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class holds configurations for the Operator Interface (so joysticks/controllers)
 */
public class OI {
    
    /** Driver Station ports controllers are attached to */
    private static class Ports {
        /** Left driving joystick */
        private static final int LEFT_STICK = 0;
        /** Right driving joystick */
        private static final int RIGHT_STICK = 1;
        /** Driving controller */
        private static final int DRIVER_CONTROLLER = 2;
        
        /** Operator controller */
        private static final int OPERATOR_CONTROLLER = 3;
    }

    /** Buttons on the driver sticks/controller */
    private static class DriverButtons {
        
    }

    /** Buttons on the operator controller */
    private static class OperatorButtons {
        /** Button used as example */
        private static final Button EXAMPLE = XboxController.Button.kA;
    }

    // This contains objects for both joystick and controller driving
    // You will uncomment code below to select the drive type you want

    /** Port for controller used by driver */
    private static final int DRIVER_CONTROLLER_PORT = 0;
    /** Port for controller used by operator */
    private static final int OPERATOR_CONTROLLER_PORT = 1;

    /** Controller used by driver, mapped to {@link #DRIVER_CONTROLLER_PORT} */
    private static final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
    /** Controller used by driver, mapped to {@link #OPERATOR_CONTROLLER_PORT} */
    private static final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);


    // The sticks/controllers are kept private so that if we want to switch them later, this is the only place needing changes
    // Use buttons and DoubleSuppliers to expose any inputs you want elsewhere
    public static double getXDriveInput(){
        return OI.driverController.getLeftX()*0.5;
    }

    public static double getYDriveInput(){
        return OI.driverController.getLeftY()*0.5;
    }

    public static double getRotateDriveInput(){
        return OI.driverController.getRightX()*0.5;
    }

}
