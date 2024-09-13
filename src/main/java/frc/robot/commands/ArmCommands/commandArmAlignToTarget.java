package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.profiledArmPID;

public class commandArmAlignToTarget extends Command {
    private final profiledArmPID arm;
    private Supplier<Double> poseSpeakerDistance;
    private Supplier<Double> cameraSpeakerDistance;

    public commandArmAlignToTarget(profiledArmPID arm, Supplier<Double> poseSpeakerDistance, Supplier<Double> cameraSpeakerDistance) {
        this.arm = arm;
        this.poseSpeakerDistance = poseSpeakerDistance;
        this.cameraSpeakerDistance = cameraSpeakerDistance;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        ShooterConstants.currentArmState = ShooterConstants.armState.SPEAKER;
        arm.enable();
    }

    @Override
    public void execute() {
        System.out.println("Difference in distance calculations: " + (poseSpeakerDistance.get() - cameraSpeakerDistance.get()));
        double calculatedArmAngle = arm.getCalculatedSpeakerAngle(poseSpeakerDistance.get()); // Shot map seems off, adjust as needed.
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