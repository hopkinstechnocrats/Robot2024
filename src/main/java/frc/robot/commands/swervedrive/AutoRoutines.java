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
        
        final PhotonCamera m_cam = new PhotonCamera("HD_Pro_Webcam_C920");
    }

    public static Command autoClimbUp(Climb climb) {
    return Commands.run(
        () -> {
          MechanismCommands.climbUp(climb).withTimeout(2);
        },
        climb);
  } 

   public static Command autoClimbDown(Climb climb) {
    return Commands.run(
        () -> {
          MechanismCommands.climbDown(climb).withTimeout(2);
        },
        climb);
  } 

   public static Command autoScore(TopArm arm, EndEffector endEffector) {
    return Commands.run(
        () -> {
          new SequentialCommandGroup(
            //m_drive.aimAtTarget(m_cam),
                MechanismCommands.armScoring(arm),
                MechanismCommands.spinBlueWheel(endEffector),
                MechanismCommands.spinRollers(endEffector)
                );       
        },
        arm, endEffector);
  }

}

