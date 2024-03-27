package frc.robot.subsystems.swervedrive.climb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberIO extends SubsystemBase{
    
    private String name;
    private double kP;
    private double kI;
    private double kD;
    public ProfiledPIDController feedback;
    private double positionOffset;

    static WPI_TalonFX motor;
    private AnalogPotentiometer stringClimber;

    public ClimberIO(String name, int motorPort, double kP, double kI, double kD, double kEncoderTicksPerRevolution, double maxVelocity, double maxAcceleration){
        this.name = name;
        stringClimber = new AnalogPotentiometer(0, 2.037, 0); //does fullRange hold true?
        kP = this.kP; 
        kI = this.kI; 
        kD = this.kD;
    
        feedback = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        motor = new WPI_TalonFX(motorPort, "GertrudeGreyser");
        positionOffset = 0;
    }

    public double getPosition() {
        return stringClimber.get()-.365;
    }

    public void setMaxSpeedAndAccel (double MaxSp, double MaxAcc) {
        feedback = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MaxSp, MaxAcc));
    }

    public void zeroPosition() {
        positionOffset = getPosition();
    }

    public void setPosition(double goal) {
        motor.set(feedback.calculate(getPosition(), goal));
    }

}
