package frc.robot.subsystems.swervedrive;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TopArmConstants;
 
public class TopArm extends SubsystemBase {
  // private static final String TopArmConstants = null;
  CANSparkMax topArmMotor;
  CANSparkMax secondArmMotor;
  SparkMaxPIDController armPIDController;
  SparkMaxPIDController armDownPIDController;

  DutyCycleEncoder absDutyCycleEncoder;
  RelativeEncoder relativeEncoder;
  double armGoalPos;
  boolean cubeMode, coneMode, reverseScore, stowPosition;
  int nodePosition;
  int storedNode;

  /** */
  public TopArm() {
    topArmMotor = new CANSparkMax(TopArmConstants.kTopArmMotorPort, MotorType.kBrushless);
    secondArmMotor = new CANSparkMax(TopArmConstants.kSecondArmMotorPort, MotorType.kBrushless);
    
    topArmMotor.restoreFactoryDefaults();
    secondArmMotor.restoreFactoryDefaults();

    secondArmMotor.setSmartCurrentLimit(40);
    topArmMotor.setSmartCurrentLimit(40);

    topArmMotor.setInverted(false);
    secondArmMotor.follow(topArmMotor,true);

    // topArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // topArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // topArmMotor.setSoftLimit(SoftLimitDirection.kReverse, -45); // -35
    // topArmMotor.setSoftLimit(SoftLimitDirection.kForward, 215);
    topArmMotor.setIdleMode(IdleMode.kBrake);
    // topArmMotor.setClosedLoopRampRate(TopArmConstants.kClosedLoopRampRate);
    // armPIDController = topArmMotor.getPIDController();

    // absDutyCycleEncoder = new DutyCycleEncoder(TopArmConstants.kTopArmEncoderPort);
    // absDutyCycleEncoder.setDistancePerRotation(360);

    relativeEncoder = topArmMotor.getEncoder();
    relativeEncoder.setPositionConversionFactor(360 / TopArmConstants.kGearRatio);

    armPIDController = topArmMotor.getPIDController();
    armPIDController.setP(TopArmConstants.kP);
    armPIDController.setI(TopArmConstants.kI);
    armPIDController.setD(TopArmConstants.kD);
    armPIDController.setIZone(TopArmConstants.kIntegralZone);
    armPIDController.setFF(TopArmConstants.kFeedForward);
    armPIDController.setOutputRange(TopArmConstants.kMinOutput, TopArmConstants.kMaxOutput);

    setTrueStowPosition();

    SmartDashboard.putNumber("Arm Goal Position", 0);

    SmartDashboard.putNumber("Top Arm kP", armPIDController.getP());
    SmartDashboard.putNumber("Top Arm kI", armPIDController.getI());
    SmartDashboard.putNumber("Top Arm kD", armPIDController.getD());
    SmartDashboard.putNumber("Top Arm IZone", armPIDController.getIZone());

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
    double angle = relativeEncoder.getPosition(); // need to divide by encoder resolution??
    angle -= TopArmConstants.kAbsEncoderOffset; // ?

    return angle * (TopArmConstants.kAbsEncoderReversed ? -1 : 1);
  }

  public void resetEncoderPosition() {
    relativeEncoder.setPosition(0);
    // relativeEncoder.setPosition(encoderPositionAngle());
  }

  public void setMotorPosition(double angle) {    
    armDownPIDController = topArmMotor.getPIDController();

    armDownPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
    armDownPIDController.setP(TopArmConstants.kP);
    armDownPIDController.setI(TopArmConstants.kI);
    armDownPIDController.setD(TopArmConstants.kD);
    armDownPIDController.setIZone(TopArmConstants.kIntegralZone);
    armDownPIDController.setFF(TopArmConstants.kFeedForward);
    armDownPIDController.setOutputRange(TopArmConstants.kMinOutput, TopArmConstants.kMaxOutput);

    armDownPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setMotorDownPosition(double angle) {
    armDownPIDController = topArmMotor.getPIDController();

    armDownPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
    armDownPIDController.setP(TopArmConstants.kPDown);
    armDownPIDController.setI(TopArmConstants.kI);
    armDownPIDController.setD(TopArmConstants.kD);
    armDownPIDController.setIZone(TopArmConstants.kIntegralZone);
    armDownPIDController.setFF(TopArmConstants.kFeedForward);
    armDownPIDController.setOutputRange(TopArmConstants.kMinOutput, TopArmConstants.kMaxOutput);

    armDownPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setArmPosition() {}

  public double getMotorEncoderPosition() {
    return relativeEncoder.getPosition();
  }

  public void setNodePosition(int node) { // Sets the node to be scored at
    nodePosition = node;
  }

  public int getNodePosition() { // Used in other places to get the node to score at
    return nodePosition;
  }

  public void setReverseScoring() {
    reverseScore = true;
  }

  public void setRegularScoring() {
    reverseScore = false;
  }

  public boolean
      getScoringPosition() { // Used in other places to dictate which side of the robot to score out
    // of
    return reverseScore;
  }

  public void stopMotors() {
    topArmMotor.stopMotor();
  }

  public void setTrueStowPosition() {
    stowPosition = true;
  }

  public void setFalseStowPositon() {
    stowPosition = false;
  }

  public boolean getStowPositionState() { // Checks if the robot is in stow position
    return stowPosition;
  }
}
