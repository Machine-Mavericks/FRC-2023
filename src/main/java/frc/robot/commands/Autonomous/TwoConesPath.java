// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConesPath extends SequentialCommandGroup {
  /** Creates a new TwoConesPath. */
  public TwoConesPath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    /**
     * drive forwards
     * place cone on high (9,2)
     * drive backwards past line
     * pick up cone 
     *    arm to positon low pick up
     *    close gripper
     *    arm to position carry cone
     * drive forward
     * place cone on mid (9,1)
     */

    );
  }
}
