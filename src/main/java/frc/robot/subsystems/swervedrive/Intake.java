package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(80, 80);
  }

  public void spinIntake() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

   public void reverseSpinIntake() {
    intakeMotor.set(-1*IntakeConstants.intakeSpeed);
  }

  public void noSpinIntake() {
    intakeMotor.set(0);
  }
}