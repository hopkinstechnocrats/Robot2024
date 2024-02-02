package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

//create constants for motor CanID and the intakeSpeed
    WPI_TalonFX intakeMotor;

    public IntakeSubsystem() {
      // BROKEN!!! FIX LATER!!!
        //intakeMotor = new WPI_TalonFX(Constants.intakeMotorCANID);
      }
    
      public void SpinIntake(boolean direction){
        // BROKEN!!! FIX LATER!!!
          //intakeMotor.set(Constants.intakeSpeed);
      }
    
      public void NoSpin(){
        intakeMotor.set(0);
      }
      
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
    
        
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
    }
