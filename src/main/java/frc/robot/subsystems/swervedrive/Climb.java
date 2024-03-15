package frc.robot.subsystems.swervedrive;


//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
//import com.ctre.phoenix6.configs.*;

public class Climb extends SubsystemBase {
  private TalonFX climbMotor;
  RelativeEncoder climbEncoder;
  PIDController pidController;

  public Climb() {
    climbMotor = new TalonFX(20, "GertrudeGreyser");
    climbMotor.setVoltage(4);
    climbMotor.setNeutralMode(NeutralModeValue.Brake);


  
  }

  public void climbUp() {
    climbMotor.set(ClimbConstants.climbSpeedUp);
  }

  public void climbDown() {
    climbMotor.set(ClimbConstants.climbSpeedDown);
  }

  public void noClimb() {
    climbMotor.set(0);
  }
}