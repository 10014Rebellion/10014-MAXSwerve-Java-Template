package frc.robot.commands.flywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        flywheels.setBothFlywheelVelocity(5800, 2000);
        ShooterConstants.currentFlywheelState = ShooterConstants.flywheelState.SHOOT;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        flywheels.setBothFlywheelVelocity(0, 0);
        ShooterConstants.currentFlywheelState = ShooterConstants.flywheelState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return false;  
    }
}
