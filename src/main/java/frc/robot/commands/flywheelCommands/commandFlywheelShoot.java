package frc.robot.commands.flywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;

public class commandFlywheelShoot extends Command{
    private final doubleShooterFlywheels flywheels;

    public commandFlywheelShoot (doubleShooterFlywheels f) {
        flywheels = f;
        addRequirements(flywheels);
    }

    @Override
    public void initialize() {
        if (IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(5800, 2900);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.SHOOT;
        }
        // else {
        //     flywheels.setBothFlywheelVelocity(0, 0);
        //     FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.NOTHING;
        // }
        
    }

    @Override
    public void execute() {
        if (IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(5800, 2900);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.SHOOT;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if ((FlywheelConstants.currentFlywheelState == FlywheelConstants.flywheelState.SHOOT)
        && IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(5800, 2900);
        }
        else if (IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(2000,2000);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.IDLE;
        }
        else {
            flywheels.setBothFlywheelVelocity(500, 500);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.NOTHING;
        }
    }

    @Override
    public boolean isFinished() {
        //return flywheels.flywheelsAtSetpoint();  
        return FlywheelConstants.flywheelsAtSetpoint;
    }
}
