package frc.robot.subsystems.mechanism;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeConstants;

public class intake {
  static CANSparkMax intakeMotor = new CANSparkMax(19, MotorType.kBrushless);

  public intake() {}

  public static void intakeSpin() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }
}
