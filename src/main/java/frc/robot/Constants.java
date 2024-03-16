// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(2000, 0.4, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0);
    //kD was 0.01
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class EndEffectorConstants {
    // check these values
    public static final int kBlueMotorPort = 28; // particularly this value
    public static final int kRollerMotorPort = 27;
    public static final double kGearRatio = 3;

    //public static final double EndEffectorSpeedIn = 0.3;
    //public static final double EndEffectorSpeedOut = -0.3;
    public static final double rollerSpeed = 0.8;
    public static final double blueWheelSpeed = -0.6;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 19;
    public static final double intakeSpeed = 1;
  }

  public static final class TopArmConstants {
    public static final int kTopArmMotorPort = 18; // not true
    public static final double kP = 0.0045;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIntegralZone = 15;
    public static final double kFeedForward = 0;
    public static final double kMinOutput = -.2; //minimized it for arm to move safelier
    public static final double kMaxOutput = 0.2;

    public static final double kPDown = .0001;
    public static final double kGearRatio = 80; // changed it because the angle was off cuz gear ratio was off

    public static final double kAbsEncoderOffset = 0;
    public static final boolean kAbsEncoderReversed = false;
    public static final double kClosedLoopRampRate = 0.5;

    public static final double kMidNodeScorePosCone = 7;
    public static final double kMidNodeScorePosCube = 5;

    public static final double kHighNodeScorePosCube = -10;
    public static final double kHighNodeScorePosCone = -33;
    public static final int kTopArmEncoderPort = 0; // for if we add an analog encoder
    public static final int kSecondArmMotorPort = 17;
  }

  public static final class ClimbConstants {
    public static final double climbSpeedUp = -0.2;
    public static final double climbSpeedDown = 1;
  }

}
