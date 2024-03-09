package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  CANSparkMax rollersMotor;
  RelativeEncoder rollersEncoder;

  NetworkTableInstance inst;
  NetworkTable table;
  NetworkTableEntry setpointLog;
  NetworkTableEntry currentVelLog;

  DigitalInput BB;
  Boolean BBState;

  public EndEffector() {

    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("EndEffector");
    setpointLog = table.getEntry("Setpoint (RPM)");
    currentVelLog = table.getEntry("Current Velocity (RPM)");

    rollersMotor = new CANSparkMax(EndEffectorConstants.kRollerMotorPort, MotorType.kBrushless);
    rollersEncoder = rollersMotor.getEncoder();
    rollersEncoder.setVelocityConversionFactor(EndEffectorConstants.kGearRatio);
    rollersMotor.setSmartCurrentLimit(15, 30);
    rollersMotor.burnFlash(); // Save settings even after brownout

    BB = new DigitalInput(Constants.EndEffectorConstants.BBDIOPort);

    blueWheelMotor = new CANSparkMax(EndEffectorConstants.kBlueMotorPort, MotorType.kBrushless);
    blueWheelEncoder = blueWheelMotor.getEncoder();
    blueWheelEncoder.setVelocityConversionFactor(EndEffectorConstants.kGearRatio);
    blueWheelMotor.setSmartCurrentLimit(15, 30);
    blueWheelMotor.burnFlash(); // Save settings even after brownout
  }

  public void spinRollers() {
    rollersMotor.set(EndEffectorConstants.rollerSpeed);
  }

  public void spinBlueWheel() {
    blueWheelMotor.set(EndEffectorConstants.blueWheelSpeed);  //TODO made a change here.  maybe wont work
  }

  public void NoSpin() {
    blueWheelMotor.set(0);
    rollersMotor.set(0);
  }

  public void NoteDetected(){
    System.out.println(BB.get());//True when sensor detects something
  }

}