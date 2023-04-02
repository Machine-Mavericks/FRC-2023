package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

// class to perform delay for provided number of seconds

public class DelayCommand extends CommandBase {
    
    private Timer m_Timer;
    private double m_delayTimer;

    // Delay command function //
    public DelayCommand(double delay) {
        m_delayTimer = delay; 
    }

    @Override
    public void initialize() {
        m_Timer = new Timer();
        m_Timer.reset();
        m_Timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Returns false when the command should end.
    @Override
    public boolean isFinished() {
        return(m_Timer.hasElapsed(m_delayTimer));
    }

    @Override
    public void end(boolean interrupted) {
    }
}