package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.profiledArmPID;

public class commandArmAutoAim extends Command {
    private final profiledArmPID arm;

    public commandArmAutoAim(profiledArmPID arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.enable();
    }

    @Override
    public void execute() {
        double calculatedArmAngle = arm.getCalculatedSpeakerAngle();
        arm.altGoToSetpoint(calculatedArmAngle);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
