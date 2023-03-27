package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
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

 
    /** Buttons on the driver controller */
    public static class DriverButtons {

        // gyro reset
        public static JoystickButton gyro_reset_Button = new JoystickButton(driverController, XboxController.Button.kBack.value);
       
        // semi-auto pickup/drop off buttons
        public static JoystickButton shelfpickup_Button = new JoystickButton(driverController, XboxController.Button.kY.value);
        public static JoystickButton DropoffHigh_Button = new JoystickButton(driverController, XboxController.Button.kX.value);
        public static JoystickButton DropoffMed_Button = new JoystickButton(driverController, XboxController.Button.kA.value);
        public static JoystickButton auto_balance_Button = new JoystickButton(driverController, XboxController.Button.kB.value);
        public static JoystickButton CubeDropoffHigh_Button = new JoystickButton(driverController, XboxController.Button.kLeftStick.value);
        public static JoystickButton CubeDropoffMed_Button = new JoystickButton(driverController, XboxController.Button.kRightStick.value);

        // park button
        public static JoystickButton park_Button = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    }


    /** Buttons on the operator controller */
    public static class OperatorButtons {
    
        // arm and grabber buttons
        public static JoystickButton ground_Button = new JoystickButton(operatorController, XboxController.Button.kA.value);
        public static JoystickButton mid_Button = new JoystickButton(operatorController, XboxController.Button.kX.value);
        public static JoystickButton high_Button = new JoystickButton(operatorController, XboxController.Button.kY.value);
        public static JoystickButton stow_Button = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
        public static JoystickButton shelf_pickup_Button = new JoystickButton(operatorController, XboxController.Button.kB.value);
        public static JoystickButton GrabberButton = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    }

    // This contains objects for both joystick and controller driving

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
        return OI.driverController.getLeftX();
    }

    public static double getYDriveInput(){
        return OI.driverController.getLeftY();
    }

    public static double getRotateDriveInput(){
        return OI.driverController.getRightX();
    }

    public static double getArmSpeed(){
        return OI.operatorController.getRightY();
    }
    public static double getGoFast (){
        return OI.driverController.getRightTriggerAxis();
    }

    public static boolean closeGripper(){
        return OI.operatorController.getLeftTriggerAxis()>=0.05;
    }
}
