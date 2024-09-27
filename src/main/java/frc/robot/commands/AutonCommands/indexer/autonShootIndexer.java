package frc.robot.commands.AutonCommands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Shooter.indexerSubsystem;

public class autonShootIndexer extends Command{
    
    private indexerSubsystem indexer;

    public autonShootIndexer(indexerSubsystem i){
        indexer = i;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.runIndexerFast();
        IndexerConstants.currentIndexState = IndexerConstants.indexState.START;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        indexer.stopIndexer();
        IndexerConstants.currentIndexState = IndexerConstants.indexState.IDLE;
    }
    @Override
    public boolean isFinished() {
        return (!IndexerConstants.robotHasNote);
    }
}
