package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Shooter.indexerSubsystem;

public class commandIndexBrakeMode extends Command{
    
    private indexerSubsystem indexer;

    public commandIndexBrakeMode(indexerSubsystem i){
        indexer = i;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.enableBrake();
        //IndexerConstants.currentIndexState = IndexerConstants.indexState.PICKUP;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        indexer.disableBrake();
        //IndexerConstants.currentIndexState = IndexerConstants.indexState.HOLD;
    }
    @Override
    public boolean isFinished() {
        return false;
        //return IndexerConstants.robotHasNote;
    }
}