package frc.robot.subsystems.Climb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem {
  
//create constants for motor CanID and the intakeSpeed
     WPI_TalonFX climbMotor;

    public ClimbSubsystem() {
      // BROKEN!!! FIX LATER!!!
        //climbMotor = new WPI_TalonFX(Constants.climbMotorCANID);
      }
    
      public void SpinClimber (boolean direction){
        // BROKEN!!! FIX LATER!!!
          //climbMotor.set(Constants.climbSpeed);
      }
    
      public void ClimbNoSpin(){
        climbMotor.set(0);
      }
      
      }  
