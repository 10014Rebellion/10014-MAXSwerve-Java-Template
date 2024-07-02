package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.shooterFlywheels;

public class flywheelCommand extends Command {

    private shooterFlywheels flywheels;

    /*public flywheelCommand(shooterFlywheels flywheels) {
        this.flywheels = flywheels;
        addRequirements(flywheels);
    }

    @Override
    public void initialize() {
       flywheels.runFlywheelForward();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.dontRunFlywheel(); 
    }

    @Override
    public boolean isFinished() {
        return true;
    }*/
}