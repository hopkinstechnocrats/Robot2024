package frc.robot.subsystems.swervedrive.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TopArmConstants;

public class ArmIO extends SubsystemBase{
    
    private String name;
    private double kP;
    private double kI;
    private double kD;
    public ProfiledPIDController feedback;
    private double positionOffset;

    static CANSparkMax leaderMotor;
    static CANSparkMax followerMotor;

    private AnalogPotentiometer stringArm;

    public ArmIO(String name, double kP, double kI, double kD, double kEncoderTicksPerRevolution, double maxVelocity, double maxAcceleration){
        this.name = name;
        stringArm = new AnalogPotentiometer(0, 2.037, 0); //does fullRange hold true?
        kP = this.kP;
        kI = this.kI;
        kD = this.kD;

        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        followerMotor.setSmartCurrentLimit(40);
        leaderMotor.setSmartCurrentLimit(40);

        leaderMotor.setInverted(false);  //TODO invert this
        followerMotor.follow(leaderMotor,true);
    
        feedback = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        leaderMotor = new CANSparkMax(TopArmConstants.kTopArmMotorPort, MotorType.kBrushless);
        followerMotor = new CANSparkMax(TopArmConstants.kSecondArmMotorPort, MotorType.kBrushless);
        positionOffset = 0;
    }

    public double getPosition() {
        return stringArm.get()-.365;
    }

    public void setMaxSpeedAndAccel (double MaxSp, double MaxAcc) {
        feedback = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MaxSp, MaxAcc));
    }

    public void zeroPosition() {
        positionOffset = getPosition();
    }

    public void setPosition(double goal) {
        leaderMotor.set(feedback.calculate(getPosition(), goal));
    }

}