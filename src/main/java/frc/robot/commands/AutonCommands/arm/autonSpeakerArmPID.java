package frc.robot.commands.AutonCommands.arm;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.profiledArmPID;
import edu.wpi.first.wpilibj2.command.Command;

public class autonSpeakerArmPID extends Command{
    private final profiledArmPID arm;

    public autonSpeakerArmPID(profiledArmPID arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.goToSetpoint(ShooterConstants.kArmSubwooferShotPosition);
        ShooterConstants.currentArmState = ShooterConstants.armState.MOVING;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        ShooterConstants.currentArmState = ShooterConstants.armState.SPEAKER;
    }

    @Override
    public boolean isFinished() {
        return ShooterConstants.armAtSetpoint;  
    }
}
