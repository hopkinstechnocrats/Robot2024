package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltraSonic extends SubsystemBase{
    
    
    DigitalInput leftSensor;
    DigitalInput rightSensor;
    
    
    public UltraSonic(){
        leftSensor = new DigitalInput(1);
        rightSensor = new DigitalInput(2);
    }

    public void PrintStatus(){
        System.out.println("Left Status: " + leftSensor.get());
        System.out.println("Right Status: " + rightSensor.get());
    }
}
