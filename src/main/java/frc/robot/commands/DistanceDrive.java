package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class DistanceDrive {

  private DistanceDrive() {}

  public static Command DistanceDrive(Drive drive) {
    return Commands.run(
        () -> {
          double speed = 0.5;
          double driveDistance = speed * 5; // five seconds of driving.
          // final Pose2d startingPose;
          // if (isFirstTime) {
          // startingPose = drive.getPose();
          // }
          double distanceTraveled = drive.getPose().getX(); // /!< TODO: This is probably wrong
          if (distanceTraveled > driveDistance) {
            speed = 0;
          }
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, 0, drive.getRotation()));
        });
  }
}
