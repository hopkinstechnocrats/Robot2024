package frc.robot.subsystems.swervedrive;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {

  CANSparkMax blueWheelMotor;
  RelativeEncoder blueWheelEncoder;
  SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  CANSparkMax rollersMotor;
  RelativeEncoder rollersEncoder;

  NetworkTableInstance inst;
  NetworkTable table;
  NetworkTableEntry setpointLog;
  NetworkTableEntry currentVelLog;

  DigitalInput BB;

  public EndEffector() {

    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("EndEffector");
    setpointLog = table.getEntry("Setpoint (RPM)");
    currentVelLog = table.getEntry("Current Velocity (RPM)");

    BB = new DigitalInput(Constants.EndEffectorConstants.BBDIOPort);

    rollersMotor = new CANSparkMax(EndEffectorConstants.kRollerMotorPort, MotorType.kBrushless);
    rollersEncoder = rollersMotor.getEncoder();
    rollersEncoder.setVelocityConversionFactor(EndEffectorConstants.kGearRatio);
    rollersMotor.setSmartCurrentLimit(40, 40);
    rollersMotor.burnFlash(); // Save settings even after brownout

    blueWheelMotor = new CANSparkMax(EndEffectorConstants.kBlueMotorPort, MotorType.kBrushless);
    blueWheelMotor.restoreFactoryDefaults();
    m_pidController = blueWheelMotor.getPIDController();
    blueWheelEncoder = blueWheelMotor.getEncoder();
    //blueWheelEncoder.setVelocityConversionFactor(EndEffectorConstants.kGearRatio);
    //blueWheelMotor.setSmartCurrentLimit(40, 40);
    //blueWheelMotor.burnFlash(); // Save settings even after brownout

    // PID coefficients
    kP = 10;//6e-5 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void spinRollers() {
    rollersMotor.set(EndEffectorConstants.rollerSpeed);
  }

  public void spinBlueWheel() {
    double setPoint = -0.6*maxRPM;
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    //blueWheelMotor.set(0.6);
  }

   public void reverseSpinRollers() {
    rollersMotor.set(-1*EndEffectorConstants.rollerSpeed);
  }

   public void reverseSpinBlueWheel() {
    double setPoint = 0.6*maxRPM;
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void NoSpin() {
    double setPoint = 0*maxRPM;
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    rollersMotor.set(0);
  }

   public BooleanSupplier NoteDetected(){
    //System.out.println(BB.get());
    return BB::get;//True when sensor detects something
    
  }

}