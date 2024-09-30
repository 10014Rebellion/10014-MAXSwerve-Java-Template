package frc.robot.commands.AutonCommands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.photonConstants;
import frc.robot.subsystems.Shooter.profiledArmPID;

public class autonAutoAimArmPID extends Command {
    private final profiledArmPID arm;
    private Supplier<Double> poseSpeakerDistance;
    private Supplier<Double> cameraSpeakerDistance;

    public autonAutoAimArmPID(profiledArmPID arm, Supplier<Double> poseSpeakerDistance, Supplier<Double> cameraSpeakerDistance) {
        this.arm = arm;
        this.poseSpeakerDistance = poseSpeakerDistance;
        this.cameraSpeakerDistance = cameraSpeakerDistance;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        ShooterConstants.currentArmState = ShooterConstants.armState.SPEAKER;
        arm.enable();
    }

    @Override
    public void execute() {
        //System.out.println("Difference in distance calculations: " + (Math.abs(poseSpeakerDistance.get()) - Math.abs(cameraSpeakerDistance.get())));
        double calculatedArmAngle = arm.getCalculatedSpeakerAngle(poseSpeakerDistance.get() - photonConstants.kCameraLocation.getX()); // Shot map seems off, adjust as needed.
        //System.out.println("Arm calculated movement: " + calculatedArmAngle);
        arm.altGoToSetpoint(calculatedArmAngle);
    }

    @Override
    public void end(boolean interrupted) {
        ShooterConstants.currentArmState = ShooterConstants.armState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return ShooterConstants.armAtSetpoint; 
    }
}