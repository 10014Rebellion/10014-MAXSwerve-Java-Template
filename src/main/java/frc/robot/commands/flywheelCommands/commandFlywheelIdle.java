package frc.robot.commands.flywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
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
        if (IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(2000, 2000);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.IDLE;
        }
        else {
            flywheels.setBothFlywheelVelocity(500, 500);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.NOTHING;
        }
        
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(1000, 1000);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.IDLE;
        }
        else {
            flywheels.setBothFlywheelVelocity(500, 500);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.NOTHING;
        }
    }

    @Override
    public boolean isFinished() {
        return false;  
    }
}
