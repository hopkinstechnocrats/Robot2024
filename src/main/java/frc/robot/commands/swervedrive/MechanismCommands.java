package frc.robot.commands.swervedrive;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.Climb;
import frc.robot.subsystems.swervedrive.EndEffector;
import frc.robot.subsystems.swervedrive.Intake;
import frc.robot.subsystems.swervedrive.TopArm;


public class MechanismCommands {
  private MechanismCommands() {}

  public static Command moveEndEffector(EndEffector endEffector) {

    return Commands.run(
        () -> {
          endEffector.NoSpin();
        },
        endEffector);
  }

  public static Command spinOut(EndEffector endEffector) {
    return Commands.run(
        () -> {
          endEffector.spinEndEffectorOut();
        },
        endEffector);
  }

  public static Command spinIn(EndEffector endEffector) {
    return Commands.run(
        () -> {
          endEffector.spinEndEffectorIn();
        },
        endEffector);
  }

  public static Command moveInIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.spinIntake();
        },
        intake);
  }

  public static Command moveNoIntake(Intake intake) {
    return Commands.run(
        () -> {
          intake.noSpinIntake();
        },
        intake);
  }

  public static Command moveArm(TopArm arm) {
    return Commands.run(
        () -> {
          arm.setMotorDownPosition(0);
        },
        arm);
  }

  public static Command moveArmFurther(TopArm arm) {
    return Commands.run(
        () -> {
          arm.setMotorDownPosition(100);
        },
        arm);
  }

  public static Command climbUp(Climb climb) {
    return Commands.run(
        () -> {
          climb.climbUp();
        },
        climb);
  }

  public static Command climbDown(Climb climb) {
    return Commands.run(
        () -> {
          climb.climbDown();
        },
        climb);
  }

  public static Command noClimb(Climb climb) {
    return Commands.run(
        () -> {
          climb.noClimb();
        },
        climb);
  }
}