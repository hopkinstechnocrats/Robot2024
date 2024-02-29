package frc.robot.subsystems.swervedrive;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private TalonFX climbMotor;

  public Climb() {
    climbMotor = new TalonFX(20, "GertrudeGreyser");
    climbMotor.setVoltage(2);
    
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