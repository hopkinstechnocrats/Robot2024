package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.EndEffector;

public class MechanismCommands {
  private MechanismCommands() {}

  public static Command moveEndEffector(EndEffector endEffector) {

    return Commands.run(
        () -> {
          endEffector.NoSpin();
        },
        endEffector);
  }

  public static Command spinPlease(EndEffector endEffector) {
    return Commands.run(
        () -> {
          endEffector.spinEndEffectorIn();
        },
        endEffector);
  }

  public static Command spinPlease2(EndEffector endEffector) {
    return Commands.run(
        () -> {
          endEffector.spinEndEffectorIn();
        },
        endEffector);
  }
}
