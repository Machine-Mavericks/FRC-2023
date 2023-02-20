package frc.robot.commands;
import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GamePieceTargeting.GamePieceData;
import frc.robot.OI;


public class ConePickupCommand extends CommandBase {

    private boolean m_Targetfound = false;
    private GamePieceData m_TargetPose;
    private double m_TargetAngle;

    private double m_AcceptableError = 0.5;
    /** Creates a new ConePickupCommand. */
    public ConePickupCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!m_Targetfound){
            if (RobotContainer.gamepiecetargeting.isTarget()){
                m_TargetPose = RobotContainer.gamepiecetargeting.getTargetPose();
                m_Targetfound = true;
                m_TargetAngle = Math.atan(m_TargetPose.m_X / m_TargetPose.m_Y);

                
                PrecisionDriveToTargetRelative drive = new PrecisionDriveToTargetRelative(new Pose2d(m_TargetPose.m_X, m_TargetPose.m_Y, new Rotation2d(m_TargetAngle)));
                drive.schedule();

            }
        }else{
            
            

            

        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
  }
}
