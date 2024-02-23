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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Run drive motor at volts */
  default void runDriveVolts(double volts) {}

  /** Run turn motor at volts */
  default void runTurnVolts(double volts) {}

  /** Run drive motor at current */
  default void runDriveCurrent(double current) {}

  /** Run turn motor at current */
  default void runTurnCurrent(double current) {}

  /** Run to drive velocity setpoint with feedforward */
  default void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {}

  /** Run to turn position setpoint */
  default void runTurnPositionSetpoint(double angleRads) {}

  default void setDrivePID(double kP, double kI, double kD) {}

  /** Configure turn PID */
  default void setTurnPID(double kP, double kI, double kD) {}
}
