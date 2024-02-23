package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector {

  CANSparkMax blueWheelMotor;
  RelativeEncoder blueWheelEncoder;

  CANSparkMax rollersMotor;
  RelativeEncoder rollersEncoder;

  NetworkTableInstance inst;
  NetworkTable table;
  NetworkTableEntry setpointLog;
  NetworkTableEntry currentVelLog;

  public EndEffector() {

    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("EndEffector");
    setpointLog = table.getEntry("Setpoint (RPM)");
    currentVelLog = table.getEntry("Current Velocity (RPM)");

    rollersMotor = new CANSparkMax(EndEffectorConstants.kMotorPort, MotorType.kBrushless);
    rollersEncoder = rollersMotor.getEncoder();
    rollersEncoder.setVelocityConversionFactor(EndEffectorConstants.kGearRatio);
    rollersMotor.setSmartCurrentLimit(15, 30);
    rollersMotor.burnFlash(); // Save settings even after brownout

    blueWheelMotor = new CANSparkMax(EndEffectorConstants.kMotorPort, MotorType.kBrushless);
    blueWheelEncoder = blueWheelMotor.getEncoder();
    blueWheelEncoder.setVelocityConversionFactor(EndEffectorConstants.kGearRatio);
    blueWheelMotor.setSmartCurrentLimit(15, 30);
    blueWheelMotor.burnFlash(); // Save settings even after brownout
  }

  public void spinEndEffectorIn() {
    spin(EndEffectorConstants.kRollersSpeedIn);
    spin(EndEffectorConstants.kBlueWheelSpeedIn);
  }

  public void spinEndEffectorOut() {
    spin(EndEffectorConstants.kRollersSpeedOut);
    spin(EndEffectorConstants.kRollersSpeedOut);
  }

  public void NoSpin() {
    blueWheelMotor.set(0);
    rollersMotor.set(0);
  }

  public void spin(double setpoint) {
    // manipulatorPID.setReference(setpoint, ControlType.kVelocity); PID
    // setpointLog.setDouble(setpoint);
    // currentVelLog.setDouble(manipulatorEncoder.getVelocity());
    blueWheelMotor.set(setpoint);
    rollersMotor.set(setpoint);
  }
}
