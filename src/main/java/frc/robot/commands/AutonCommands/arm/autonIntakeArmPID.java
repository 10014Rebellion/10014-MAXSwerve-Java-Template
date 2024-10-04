package frc.robot.commands.AutonCommands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.profiledArmPID;

public class autonIntakeArmPID extends Command{
    private final profiledArmPID arm;

    public autonIntakeArmPID(profiledArmPID arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.goToSetpoint(ShooterConstants.kArmIntakePosition);
        ShooterConstants.currentArmState = ShooterConstants.armState.MOVING;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        ShooterConstants.currentArmState = ShooterConstants.armState.INTAKE;
    }

    @Override
    public boolean isFinished() {
        return ShooterConstants.armAtSetpoint;  
    }
}