package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.Shooter.intakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;;

public class commandIntakePickup extends Command{
    private final intakeSubsystem intake;

    public commandIntakePickup(intakeSubsystem i) {
        intake = i;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (!IndexerConstants.robotHasNote){
            intake.runIntake();
        }
        else {
            intake.stopIntake();
        }
        IndexerConstants.currentIntakeState = IndexerConstants.intakeState.PICKUP;
    }

    @Override
    public void execute() {
        if (!IndexerConstants.robotHasNote){
            intake.runIntake();
        }
        else {
            intake.stopIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        IndexerConstants.currentIntakeState = IndexerConstants.intakeState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return IndexerConstants.robotHasNote;
    }
}
