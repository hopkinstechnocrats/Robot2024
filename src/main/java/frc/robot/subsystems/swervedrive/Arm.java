package frc.robot.subsystems.swervedrive;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
 
public class Arm extends SubsystemBase {
  // private static final String TopArmConstants = null;
  private static final int leaderDeviceID = 18;
  private static final int followerDeviceID = 17;
  private CANSparkMax m_leaderMotor;
  private CANSparkMax m_followerMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
  /** */
  public Arm() {
    m_leaderMotor = new CANSparkMax(leaderDeviceID, MotorType.kBrushless);
    m_followerMotor = new CANSparkMax(followerDeviceID, MotorType.kBrushless);

    m_leaderMotor.restoreFactoryDefaults();
    m_followerMotor.restoreFactoryDefaults();

    m_followerMotor.setSmartCurrentLimit(40);
    m_leaderMotor.setSmartCurrentLimit(40);

    m_leaderMotor.setInverted(false);
    m_followerMotor.follow(m_leaderMotor,true);
    m_leaderMotor.setIdleMode(IdleMode.kBrake);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */

    // initialze PID controller and encoder objects
    m_pidController = m_leaderMotor.getPIDController();
    m_encoder = m_leaderMotor.getEncoder();
    //m_encoder.setPositionConversionFactor(360/ArmConstants.kGearRatio);

    // PID coefficients
    kP = ArmConstants.kP; 
    kI = ArmConstants.kI;
    kD = ArmConstants.kD; 
    kIz = ArmConstants.kIz; 
    kFF = ArmConstants.kFF; 
    kMaxOutput = ArmConstants.kMaxOutput; 
    kMinOutput = ArmConstants.kMinOutput;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 8000; // rpm
    maxAcc = 8000;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);


    new Thread(
            () -> {
              try {
                Thread.sleep(2000);
                resetEncoderPosition();
              } catch (Exception e) {
              }
            })
        .start();
  }

  @Override
  public void periodic() {
    /*  This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Motor Encoder Position", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Top Absolute Encoder Position", encoderPositionAngle());
    SmartDashboard.putNumber(
        "Top Absolute Position", absDutyCycleEncoder.getAbsolutePosition() * 360);
    SmartDashboard.putNumber("raw Absolute Encoder", absDutyCycleEncoder.get());
    SmartDashboard.putBoolean("Reverse Scoring", reverseScore);
    SmartDashboard.putNumber("Top Arm Output", topArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("TopArmNodePos", getNodePosition());
    armGoalPos = SmartDashboard.getNumber("Arm Goal Position", 0);
    SmartDashboard.putBoolean("Stow Position", getStowPositionState());

    armPIDController.setP(SmartDashboard.getNumber("Top Arm kP", armPIDController.getP()));
    armPIDController.setI(SmartDashboard.getNumber("Top Arm kI", armPIDController.getI()));
    armPIDController.setD(SmartDashboard.getNumber("Top Arm kD", armPIDController.getD()));
    armPIDController.setIZone(
        SmartDashboard.getNumber("Top Arm IZone", armPIDController.getIZone()));

    SmartDashboard.putNumber("Acutal Top Arm kP", armPIDController.getP()); */

  }

  public Double encoderPositionAngle() {
    double angle = m_encoder.getPosition(); 
    angle -= ArmConstants.kAbsEncoderOffset; 

    return angle * (ArmConstants.kAbsEncoderReversed ? -1 : 1);
  }

  public void resetEncoderPosition() {
    m_encoder.setPosition(0);
    // relativeEncoder.setPosition(encoderPositionAngle());
  }

  public void setMotorPosition(double angle) {    
    m_pidController.setReference(angle * ArmConstants.kGearRatio / 360.0, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setMotorDownPosition(double angle) {
    m_pidController.setReference(angle * ArmConstants.kGearRatio / 360.0, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setArmPosition() {}

  public double getMotorEncoderPosition() {
    return m_encoder.getPosition();
  }

  public void stopMotors() {
    m_leaderMotor.stopMotor();
  }

}
