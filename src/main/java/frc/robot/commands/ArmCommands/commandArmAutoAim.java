package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.profiledArmPID;

public class commandArmAutoAim extends Command {
    private final profiledArmPID arm;

    public commandArmAutoAim(profiledArmPID arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        ShooterConstants.currentArmState = ShooterConstants.armState.MOVING;
        arm.enable();
    }

    @Override
    public void execute() {
        double calculatedArmAngle = arm.getCalculatedSpeakerAngle(); // Shot map seems off, adjust as needed.
        arm.altGoToSetpoint(calculatedArmAngle);
    }

    @Override
    public void end(boolean interrupted) {
        ShooterConstants.currentArmState = ShooterConstants.armState.SPEAKER;
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
