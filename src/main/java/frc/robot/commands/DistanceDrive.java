package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;

/*needed things:
getSelectedSensorposition() * DriveConstants.kEncoderDistancePerPulse;
*/

public class DistanceDrive {

  public DistanceDrive(Drive drive, Module module) {}

  public void getSelectedSensorposition() {}

  // inputs front left driver and encoders
  public static Command distanceDrive(Drive drive, Module module) {
    Module m_module = module;

    double positionMeters = m_module.getPositionMeters(); // drive encoder
    Rotation2d rotationRadians = m_module.getAngle(); // turn encoder
    // double rotationRadiansDouble = rotationRadians.getRadians();

    double y_distance_moved = positionMeters * rotationRadians.getSin();
    double x_distance_moved = positionMeters * rotationRadians.getCos();

    System.out.println(x_distance_moved);
    System.out.println(y_distance_moved);

    return null;
  }
}
