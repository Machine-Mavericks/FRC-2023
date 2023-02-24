// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LEDBlinkin extends SubsystemBase {

  //Victor led;
  PWM led;
  /** Creates a new LEDBlinkin. */
  public LEDBlinkin() {

      // set up pwm channel
      //led = new Victor(RobotMap.PWMPorts.LED_BLINKIN);
      led = new PWM(RobotMap.PWMPorts.LED_BLINKIN);
      
      setPattern(LED_PATTERN.TEST);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Since LED comand dont like working we are doing it here


    //Low power-owerr-werrr-errrr-rrrrr
    //if( !(RobotContainer.panel.getVoltage() <11.0))
   // {
    //  RobotContainer.LEDStrip.setPattern(LED_PATTERN.LOWBATTERY);
   // }

   //Working????
    //System.out.println("************************LED COMAND************************************************************");

    RobotContainer.LEDStrip.setPattern(LED_PATTERN.TEST);
    // If yes YAY if no heh have fun fixing
  }

  public enum LED_PATTERN {
    OFF,
    REDBALL,
    BLUEBALL,
    HUB,
    LOWBATTERY,
    DISCO,
    TEST
  };

  // sets pattern of LED strip
  public void setPattern(LED_PATTERN pattern)
  {
    switch (pattern) {
      case OFF:
        led.setSpeed(0.99);    // black
      break;
      case REDBALL:
        led.setSpeed(-0.11);   // strobe red
      break;
      case BLUEBALL:
        led.setSpeed(-0.09);   // strobe blue
      break;
      case HUB:
        led.setSpeed(-0.07);   // strobe red
      break;
      case LOWBATTERY:
        led.setSpeed(0.91);    // solid purple
      break;
      case DISCO:
        led.setSpeed(-0.45);   // color wave - rainbow
      break;
      case TEST:
        led.setSpeed(-0.35);
      break;

    }
    
  }


}
