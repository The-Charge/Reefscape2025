package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble extends Command {
    private CommandXboxController xboxController;
    private RumbleType type;
    private double value;

    public Rumble(CommandXboxController xboxController, RumbleType type, double value) {
        this.xboxController = xboxController;
        this.type = type;
        this.value = value;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        xboxController.setRumble(type, value);
    }

    @Override
    public void end(boolean interrupted) {
        xboxController.setRumble(type, 0);
    }
}
