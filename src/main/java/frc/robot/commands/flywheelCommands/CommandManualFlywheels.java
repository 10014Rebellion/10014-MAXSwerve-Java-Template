package frc.robot.commands.flywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;

public class CommandManualFlywheels extends Command {
    private final doubleShooterFlywheels flywheels;
    private final double flywheelVoltage;

    public CommandManualFlywheels (doubleShooterFlywheels f, double fVoltage) {
        flywheels = f;
        flywheelVoltage = fVoltage;
        addRequirements(flywheels);
    }

    @Override
    public void initialize() {
        flywheels.setFlywheelVoltage(12);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        flywheels.setFlywheelVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;  
    }

}
