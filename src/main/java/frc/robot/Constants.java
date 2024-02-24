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

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;
  public static final boolean tuningMode = true;

  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 6;
    public static final int kRearLeftDriveMotorPort = 10;
    public static final int kFrontRightDriveMotorPort = 8;
    public static final int kRearRightDriveMotorPort = 12;

    public static final double kDriveKP = 0.1; // good number
    public static final double kDriveKI = 0.01;
    public static final double kDriveKD = 0;

    public static final double kTurnKP = 1;
    public static final double kTurnKI = 0;
    public static final double kTurnKD = 0;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kRearLeftTurningMotorPort = 11;
    public static final int kFrontRightTurningMotorPort = 9;
    public static final int kRearRightTurningMotorPort = 13;

    public static final int kFrontLeftTurningEncoderPort = 2;
    public static final int kRearLeftTurningEncoderPort = 3;
    public static final int kFrontRightTurningEncoderPort = 0;
    public static final int kRearRightTurningEncoderPort = 1;

    // rotations
    public static final double kFrontRightOffset = (2.321516939263821 + Math.PI);
    public static final double kFrontLeftOffset = 0.34651825800256497;
    public static final double kRearRightOffset = 0.3403026838679898;
    public static final double kRearLeftOffset = (-1.725615573002353 + Math.PI);
  }

  /* Turn position 0 = -0.09177295726990496
  turn Absolute position 0 = 2.784981618673524
  Turn position 1 = 2.9708094592051917
  turn Absolute position 1 = 0.35012567346983686
  Turn position 2 = 3.0972299292041408
  turn Absolute position 2 = 2.8707491349248175
  Turn position 3 = -1.0753614384621781
  turn Absolute position 3 = 0.8750591080688083
  */

  // dont worry about that
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
