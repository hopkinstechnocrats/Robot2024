package frc.robot.subsystems.swervedrive.climb;

import com.ctre.phoenix.Logger;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {

    public ClimberIO m_climberIO = new ClimberIO("Climb", 20, 7, 0.7, 0, 2048, .9, 1.25);
    //ClosedLoopIOInputs inputs;
    int speed = 0;

    public Climber() {
        //inputs = new ClosedLoopIOInputs(1);
    }

    public void spinClimber(double speed) {
        m_climberIO.motor.setVoltage(speed);
    }
    public void goTo(double Pos){
        m_climberIO.setPosition(Pos);
    }

    /*public void periodic() {
        double startTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("Climb/positionMeters", m_climberIO.getPosition());
        Logger.getInstance().recordOutput("Climb/positionErrorMeters", m_climberIO.feedback.getPositionError());
        //Logger.getInstance().processInputs("Climb", inputs);
        //m_climberIO.updateInputs(inputs);
        Logger.getInstance().recordOutput("Climb/goalMeters", m_climberIO.feedback.getGoal().position);
        Logger.getInstance().recordOutput("Climb/setpointMeters", m_climberIO.feedback.getSetpoint().position);
        double endTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("ClimberCodeSec", endTime-startTime);
    }*/ 
    //TO DO: Look into this. Can it be applied here?

    public void zeroClimberPosition() {
        m_climberIO.zeroPosition();
    }

    public void resetController() {
        m_climberIO.feedback.reset(m_climberIO.getPosition());
    }

    public double getPosition() {
        return m_climberIO.getPosition();
    }

    public void setPosition(double goal){
        m_climberIO.setPosition(goal);
    }

}

