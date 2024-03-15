package frc.robot.subsystems.swervedrive.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    public ArmIO m_armIO = new ArmIO("Climb", 7, 0.7, 0, 2048, .9, 1.25);
    //ClosedLoopIOInputs inputs;
    int speed = 0;

    public ArmSubsystem() {
        //inputs = new ClosedLoopIOInputs(1);
    }

    public void spinClimber(double speed) {
        m_armIO.leaderMotor.setVoltage(speed);
    }
    public void goTo(double Pos){
        m_armIO.setPosition(Pos);
    }

    /*public void periodic() {
        double startTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("Climb/positionMeters", m_climberIO.getPosition());
        Logger.getInstance().recordOutput("Climb/positionErrorMeters", m_climberIO.feedback.getPositionError());
        Logger.getInstance().processInputs("Climb", inputs);
        m_climberIO.updateInputs(inputs);
        Logger.getInstance().recordOutput("Climb/goalMeters", m_climberIO.feedback.getGoal().position);
        Logger.getInstance().recordOutput("Climb/setpointMeters", m_climberIO.feedback.getSetpoint().position);
        double endTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("ClimberCodeSec", endTime-startTime);
    }*/

    public void zeroClimberPosition() {
        m_armIO.zeroPosition();
    }

    public void resetController() {
        m_armIO.feedback.reset(m_armIO.getPosition());
    }

    public double getPosition() {
        return m_armIO.getPosition();
    }

    public void setPosition(double goal){
        m_armIO.setPosition(goal);
    }
 
}
