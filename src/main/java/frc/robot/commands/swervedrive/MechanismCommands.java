package frc.robot.commands.swervedrive;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.Arm;
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

  public static Command spinBlueWheel(EndEffector endEffector) {
    return Commands.run(
        () -> {
          endEffector.spinBlueWheel();
        },
        endEffector);
  }

  public static Command spinRollers(EndEffector endEffector) {
    return Commands.run(
        () -> {
          endEffector.spinRollers();
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

  public static Command armZero(TopArm arm) {
    return Commands.run(
        () -> {
          arm.setMotorPosition(110); //TO DO: test
        },
        arm);
  }

  public static Command armScoringLess(TopArm arm) {
    return Commands.run(
        () -> {
          arm.setMotorPosition(110); //TO DO: test
        },
        arm);
  }

  public static Command armScoring(TopArm arm) {
    return Commands.run(
        () -> {
          arm.setMotorPosition(115); //TO DO: test
        },
        arm);
  }

  public static Command armScoringMore(TopArm arm) {
    return Commands.run(
        () -> {
          arm.setMotorPosition(120); //TO DO: test
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

  public static Command reverseEverything(EndEffector endEffector, Intake intake) {
    return Commands.run(
        () -> {
          intake.reverseSpinIntake();
          endEffector.reverseSpinBlueWheel();
          endEffector.reverseSpinRollers();
        },
        intake, endEffector);
  }

  public static Command spinIntakeAndRollers(Intake intake, EndEffector endEffector) {
    return Commands.run(
        () -> {
          intake.spinIntake();
          endEffector.spinRollers();
        },
        endEffector, intake);
  }

  public static Command sendIt(EndEffector endEffector) {
    return Commands.run(
        () -> {
          endEffector.spinRollers();
          endEffector.spinBlueWheel();
        },
        endEffector);
  }

  public static Command newArmPosition(Arm arm) {
    return Commands.run(
        () -> {
          arm.moveArmToPosition();
        },
        arm);
  }


}