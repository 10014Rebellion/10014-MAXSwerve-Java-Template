package frc.robot.commands.AutonCommands.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.indexerSubsystem;

public class autonShootIndexer extends Command{
    
    private indexerSubsystem indexer;
    private Debouncer beamBrakeDebounce = new Debouncer(0.25);
    public autonShootIndexer(indexerSubsystem i){
        indexer = i;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        if (ShooterConstants.armAtSetpoint) {
            indexer.runIndexerFast();
        }
        IndexerConstants.currentIndexState = IndexerConstants.indexState.START;
    }

    @Override
    public void execute() {
        if (ShooterConstants.armAtSetpoint) {
            indexer.runIndexerFast();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIndexer();
        IndexerConstants.currentIndexState = IndexerConstants.indexState.IDLE;
    }
    @Override
    public boolean isFinished() {
        return (beamBrakeDebounce.calculate(!IndexerConstants.robotHasNote)); //&& ShooterConstants.armAtSetpoint);
    }
}
