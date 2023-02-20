// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDBlinkin.LED_PATTERN;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoopCubePath extends SequentialCommandGroup {
  /** Creates a new CoopCubePath. */
  public CoopCubePath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new AutoDelayCommand(),
    new InstantCommand(() -> RobotContainer.LEDStrip.setPattern(LED_PATTERN.BLUEBALL)));
    /**
     * drive forwards
     * place cube on high (5,2)
     * drive backwards over charging station past line
     * drive forwards onto charging station
     * balance/dock
     */

    
  }
}
