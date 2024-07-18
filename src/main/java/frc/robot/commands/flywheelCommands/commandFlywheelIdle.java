package frc.robot.commands.flywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;

public class commandFlywheelIdle extends Command{
    private final doubleShooterFlywheels flywheels;

    public commandFlywheelIdle (doubleShooterFlywheels f) {
        flywheels = f;
        addRequirements(flywheels);
    }

    @Override
    public void initialize() {
        flywheels.setBothFlywheelVelocity(1000, 1000);
        ShooterConstants.currentFlywheelState = ShooterConstants.flywheelState.IDLE;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        flywheels.setBothFlywheelVelocity(1000, 1000);
        ShooterConstants.currentFlywheelState = ShooterConstants.flywheelState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return false;  
    }
}
