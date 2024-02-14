package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class DistanceDrive {

  private DistanceDrive() {}

  public static Command distanceDrive(Drive drive, Encoder encoder) {
    return Commands.run(
        () -> {
          double speed = 0.5;
          double driveDistance = speed * 5; // five seconds of driving
          double distanceTraveled = Drive.encoder.getDistance();

          while (distanceTraveled < driveDistance) {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, 0, drive.getRotation()));
          }
        });
  }
}
