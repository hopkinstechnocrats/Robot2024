package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;

public class Arm {
  CANSparkMax armLeaderMotor;
  RelativeEncoder armLeaderEncoder;

  CANSparkMax armFollowerMotor;
  RelativeEncoder armFollowerEncoder;



  NetworkTableInstance inst;
  NetworkTable table;
  NetworkTableEntry setpointLog;
  NetworkTableEntry currentVelLog;

  public Arm(){
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("EndEffector");
    setpointLog = table.getEntry("Setpoint (RPM)");
    currentVelLog = table.getEntry("Current Velocity (RPM)");

    armLeaderMotor = new CANSparkMax(EndEffectorConstants.kMotorPort, MotorType.kBrushless);
    armLeaderEncoder = armLeaderMotor.getEncoder();
    armLeaderEncoder.setVelocityConversionFactor(EndEffectorConstants.kGearRatio);
    armLeaderMotor.setSmartCurrentLimit(15, 30);
    armLeaderMotor.burnFlash(); // Save settings even after brownout

    armFollowerMotor.follow(armLeaderMotor);
    //armFollowerMotor.setInverted(true); do we even need to?
  }

  public void moveArm(){
    armLeaderMotor.set(ArmConstants.armSpeed);
 }
}
