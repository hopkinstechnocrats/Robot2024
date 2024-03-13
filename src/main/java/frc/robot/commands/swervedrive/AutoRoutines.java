package frc.robot.commands.swervedrive;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.Climb;
import frc.robot.subsystems.swervedrive.EndEffector;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.TopArm;

public class AutoRoutines {
    public AutoRoutines(){
        
      //final PhotonCamera m_cam = new PhotonCamera("HD_Pro_Webcam_C920");
    }

   public static Command autoScore(TopArm arm, EndEffector endEffector) {
    return Commands.run(
        () -> {
          new SequentialCommandGroup(
                MechanismCommands.armScoring(arm),
                MechanismCommands.spinBlueWheel(endEffector),
                MechanismCommands.spinRollers(endEffector)
                );       
        },
        arm, endEffector);
  }

}

