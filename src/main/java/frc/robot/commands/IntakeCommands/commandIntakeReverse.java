package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.Shooter.intakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;;

public class commandIntakeReverse extends Command{
    private final intakeSubsystem intake;

    public commandIntakeReverse(intakeSubsystem i) {
        intake = i;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.reverseIntake();
        IndexerConstants.currentIntakeState = IndexerConstants.intakeState.REVERSE;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        IndexerConstants.currentIntakeState = IndexerConstants.intakeState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}