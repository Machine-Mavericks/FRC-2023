// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LEDBlinkin extends SubsystemBase {

  // Victor led;
  PWM led;

  /** Creates a new LEDBlinkin. */
  public LEDBlinkin() {

    // set up pwm channel
    // led = new Victor(RobotMap.PWMPorts.LED_BLINKIN);
    led = new PWM(RobotMap.PWMPorts.LED_BLINKIN);

    setPattern(LED_PATTERN.TEST);
  }

  @Override
  public void periodic() {

    if (RobotContainer.limelight_med.isTargetPresent() == 1.0)
      setPattern(LED_PATTERN.STROBEBLUE);
    else if (RobotContainer.limelight_high.isTargetPresent() == 1.0)
      setPattern(LED_PATTERN.FLASHYELLOW);
    else
      setPattern(LED_PATTERN.HEARTBEATRED);

    /*
     * if (RobotContainer.targetselector.isConeSelected())
     * {
     * // we are in cone mode
     * if (RobotContainer.limelight_high.isTargetPresent()==1.0)
     * setPattern(LED_PATTERN.SOLIDYELLOW);
     * else
     * setPattern(LED_PATTERN.FLASHYELLOW);
     * }
     * else
     * {
     * //cone mode is selected
     * if (RobotContainer.limelight_high.isTargetPresent()==1.0)
     * setPattern(LED_PATTERN.SOLIDVIOLET);
     * else
     * setPattern(LED_PATTERN.FLASHVIOLET);
     * }
     */

  }

  public enum LED_PATTERN {
    OFF,
    LOWBATTERY,
    TEST,
    SOLIDYELLOW,
    FLASHYELLOW,
    SOLIDVIOLET,
    FLASHVIOLET,
    STROBEBLUE,
    HEARTBEATRED
  };

  // sets pattern of LED strip
  /**
   * SET
   * 
   * @param pattern
   */
  public void setPattern(LED_PATTERN pattern) {
    switch (pattern) {
      case OFF:
        led.setSpeed(0.99); // black
        break;
      case LOWBATTERY:
        led.setSpeed(0.91); // solid purple
        break;
      case SOLIDYELLOW:
        led.setSpeed(0.69);
        break;
      case FLASHYELLOW:
        led.setSpeed(-0.07);
        break;
      case SOLIDVIOLET:
        led.setSpeed(0.89);
        break;
      case FLASHVIOLET:
        led.setSpeed(-0.09);
        break;
      case STROBEBLUE:
        led.setSpeed(-0.09);
        break;
      case HEARTBEATRED:
        led.setSpeed(-0.25);
      case TEST:
        led.setSpeed(-0.35);
        break;

    }

  }

}
