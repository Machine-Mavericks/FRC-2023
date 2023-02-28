// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidPath extends SequentialCommandGroup {
  /** Creates a new MidPath. */
  public MidPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    /**
     drive forward
     place cone on 9,2
      if no place game piece command
        move arm to out position for high pole
        drop game piece (ie. open grabber)
        retract arm
     drive backward past charging station
     drive forwards onto station
     dock/balance
     */

    );
  }
}
