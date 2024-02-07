package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;

public class GyroIONavX implements GyroIO {
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);
  private final float yaw = m_gyro.getYaw();

  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(yaw);
  }
}
