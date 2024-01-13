package lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class WaitCommand extends Command {
    final JoystickButton button;

    public WaitCommand(JoystickButton button) {
        // variable = expression;
        this.button = button;
    }

    public boolean isFinished() {
        return button.getAsBoolean();
    }
}