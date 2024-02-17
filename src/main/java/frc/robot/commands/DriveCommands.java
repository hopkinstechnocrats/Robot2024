// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PhotonVision;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  static PhotonVision m_PhotonVision = new PhotonVision();
  
  private static double m_yawCurrent = 0.0; ///!< From NavX
  private static double m_yawTarget = 0.0; ///!< From apriltag
  private static double m_depthCurrent = 0.0; ///!< From NavX
  private static double m_depthTarget; ///!< From apriltag
  private static boolean m_haveTarget = false; ///!< Have we seen an april tag lately
  static AHRS ahrs;



  private DriveCommands() {
     ahrs = new AHRS(SPI.Port.kMXP);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);}



  public static Command VisionDrive(
    Drive drive,
    double driveforward,
    double turnrate
     ) 
  
  {
    //x for april tag is equal to y for navx.
    Transform3d Actual_TF = m_PhotonVision.GetCamData();
    double x_actual = Actual_TF.getX();
    double y_actual = Actual_TF.getY();
    double z_actual = Actual_TF.getZ();
    double kp_x = 1.0;
    double kp_y = 10.0;
    //use wheel encoders?? using navx for now.
    m_yawCurrent = Math.toRadians(ahrs.getAngle());
    m_depthCurrent = ahrs.getDisplacementY();

    boolean haveNewData = x_actual != 0 && y_actual != 0;
    
    if (haveNewData){
      m_haveTarget = true;
      m_yawTarget = m_yawCurrent + Math.atan2(y_actual, x_actual);
      m_depthTarget = m_depthCurrent + x_actual;

      System.out.println(m_yawCurrent);
      System.out.println(m_depthCurrent);
      
      //wait
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        Thread.currentThread().interrupt();
      }
    } else {
      // TODO: If it's been too long since we saw anything, maybe give up?
      System.out.println("no april tag seen!");
    }
    
    if(m_haveTarget || true)
    {
      final double speedCap = 0.5;
      driveforward = (m_depthCurrent - m_depthTarget)*kp_x;
      turnrate = (m_yawCurrent - m_yawTarget)*kp_y;
      if(turnrate >  speedCap){turnrate =  speedCap;}
      if(turnrate < -speedCap){turnrate = -speedCap;}
      //can't be arcade drive, need to create a method for driving a distance
      
      drive.XXXX;
    
    }else{

    }
   
   
    return null;

  }
}
