package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.indexerSubsystem;

public class commandRunIndexer extends Command{
    
    private indexerSubsystem indexer;

    public commandRunIndexer(indexerSubsystem i){
        indexer = i;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.runIndexer();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        indexer.stopIndexer();
        System.out.println("why did i stop im very confused.");
    }
    @Override
    public boolean isFinished() {
        System.out.println("Unga bunga?");
        return false;
        
    }
}
