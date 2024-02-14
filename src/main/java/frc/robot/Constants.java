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

  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 6;
    public static final int kRearLeftDriveMotorPort = 10;
    public static final int kFrontRightDriveMotorPort = 8;
    public static final int kRearRightDriveMotorPort = 12;

    public static final double kDriveKP = 0.085; // was 0.15
    public static final double kDriveKI = 0;
    public static final double kDriveKD = 0;

    public static final double kTurnKP = 0; // was .5
    public static final double kTurnKI = 0;
    public static final double kTurnKD = 0;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kRearLeftTurningMotorPort = 11;
    public static final int kFrontRightTurningMotorPort = 9;
    public static final int kRearRightTurningMotorPort = 13;

    public static final int kFrontLeftTurningEncoderPort = 2;
    public static final int kRearLeftTurningEncoderPort = 3;
    public static final int kFrontRightTurningEncoderPort = 1;
    public static final int kRearRightTurningEncoderPort = 0;

    public static final double kFrontRightOffset = 0.5046380758756043;
    public static final double kFrontLeftOffset = 0.6419277889797301;
    public static final double kRearRightOffset = 0.49709956856856613;
    public static final double kRearLeftOffset = 0.7301865723975614;
  }
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
