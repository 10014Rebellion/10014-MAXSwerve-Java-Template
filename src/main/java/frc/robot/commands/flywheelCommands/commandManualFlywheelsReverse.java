package frc.robot.commands.flywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;

public class commandManualFlywheelsReverse extends Command {
    private final doubleShooterFlywheels flywheels;

    public commandManualFlywheelsReverse (doubleShooterFlywheels f) {
        flywheels = f;
        addRequirements(flywheels);
    }

    @Override
    public void initialize() {
        flywheels.setFlywheelVoltage(-6);
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