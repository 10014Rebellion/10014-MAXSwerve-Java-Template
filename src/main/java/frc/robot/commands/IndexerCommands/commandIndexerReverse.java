package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Shooter.indexerSubsystem;

public class commandIndexerReverse extends Command{
    
    private indexerSubsystem indexer;

    public commandIndexerReverse(indexerSubsystem i){
        indexer = i;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.reverseIndexer();
        IndexerConstants.currentIndexState = IndexerConstants.indexState.REVERSE;
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
        return false;
        
    }
}