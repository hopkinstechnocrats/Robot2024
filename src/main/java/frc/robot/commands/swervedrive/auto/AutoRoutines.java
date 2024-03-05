package frc.robot.commands.swervedrive.auto;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.MechanismCommands;
import frc.robot.subsystems.swervedrive.Arm;
import frc.robot.subsystems.swervedrive.Climb;
import frc.robot.subsystems.swervedrive.EndEffector;
import frc.robot.subsystems.swervedrive.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoRoutines{
    public final static PhotonCamera m_cam = new PhotonCamera("HD_Pro_Webcam_C920");
    public AutoRoutines(){
        
    }
        
   public static Command autoClimb(Climb climb, Arm arm, SwerveSubsystem m_drive) {
    return Commands.run(
        () -> {
          new SequentialCommandGroup(
            m_drive.aimAtTarget(m_cam),
            MechanismCommands.armFar(arm).withTimeout(3),
            MechanismCommands.climbUp(climb).withTimeout(3),
            MechanismCommands.climbDown(climb).withTimeout(3),
            MechanismCommands.armStraight(arm)            
            );
        },
        climb, arm);
  }

  public static Command autoScore(Arm arm, EndEffector endEffector, SwerveSubsystem m_drive) {
    return Commands.run(
        () -> {
          new SequentialCommandGroup(
            m_drive.aimAtTarget(m_cam),
            MechanismCommands.armScoring(arm),
            new ParallelCommandGroup(
                MechanismCommands.spinBlueWheel(endEffector),
                MechanismCommands.spinRollers(endEffector)
                )       
            );
        },
        arm, endEffector);
  }


}
