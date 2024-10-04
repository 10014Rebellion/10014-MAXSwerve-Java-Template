package frc.robot.commands.AutonCommands.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Shooter.indexerSubsystem;

public class autonPickupIndexer extends Command{
    
    private indexerSubsystem indexer;
    private Debouncer beamBrakeDebounce = new Debouncer(0.5);

    public autonPickupIndexer(indexerSubsystem i){
        indexer = i;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        if (!IndexerConstants.robotHasNote) {
            indexer.runIndexer();
        }
        else {
            indexer.stopIndexer();
        }
        IndexerConstants.currentIndexState = IndexerConstants.indexState.PICKUP;
    }

    @Override
    public void execute() {
        if (!IndexerConstants.robotHasNote) {
            indexer.runIndexer();
        }
        else {
            indexer.stopIndexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIndexer();
        IndexerConstants.currentIndexState = IndexerConstants.indexState.HOLD;
    }
    @Override
    public boolean isFinished() {
        return beamBrakeDebounce.calculate(IndexerConstants.robotHasNote);
    }
}