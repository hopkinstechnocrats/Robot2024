package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbServoConstants;

public class ClimbServo extends SubsystemBase {
    
    private Servo climbLocker;

    public ClimbServo(){
    
        climbLocker = new Servo(0);
		climbLocker.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    }

    public void servoLock(double position){
        climbLocker.set(position);
    }

    public void servoUnlock(){
        climbLocker.set(0.05);
    }


}
