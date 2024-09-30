package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.profiledArmPID;

public class commandArmAmp extends Command{
    private final profiledArmPID arm;
    private boolean mLess = false;

    public commandArmAmp(profiledArmPID arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    public commandArmAmp(profiledArmPID arm, boolean less) {
        this.arm = arm;
        mLess = less;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.goToSetpoint(ShooterConstants.kArmAmpPosition);// - ((mLess) ? 8 : 0));
        ShooterConstants.currentArmState = ShooterConstants.armState.MOVING;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        ShooterConstants.currentArmState = ShooterConstants.armState.AMP;
    }

    @Override
    public boolean isFinished() {
        return ShooterConstants.armAtSetpoint;  
    }
}