package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.climbState;
import frc.robot.subsystems.pidClimbSubsystem;

public class commandClimbAutoZero extends Command {
    private pidClimbSubsystem climb;

    public commandClimbAutoZero(pidClimbSubsystem c) {
        climb = c;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.moveLeftClimb(0.1);
        climb.moveRightClimb(0.1);
        ClimbConstants.currentClimbState = climbState.ZEROING;
    }

    @Override
    public void execute() {
        if(climb.getLeftBottomSensor()) {
            climb.moveLeftClimb(0);
        }
        if(climb.getRightBottomSensor()) {
            climb.moveRightClimb(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climb.moveLeftClimb(0);
        climb.moveRightClimb(0);
        ClimbConstants.currentClimbState = climbState.ATZERO;
    }
    
    
}
