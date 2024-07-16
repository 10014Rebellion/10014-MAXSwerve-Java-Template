package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class indexerSubsystem extends SubsystemBase{
    public CANSparkMax indexerMotor;
    //public DigitalInput noteDetector = new DigitalInput(0);
    public double indexerSpeed = 1.5;

    public indexerSubsystem() {
        indexerMotor = new CANSparkMax(ShooterConstants.kIndexerMotorCanID, MotorType.kBrushless);
        indexerMotor.setIdleMode(IdleMode.kCoast);
        indexerMotor.setInverted(true);
        indexerMotor.setSmartCurrentLimit(ShooterConstants.kIndexerMotorCurrentLimit);
    }

    public void runIndexer() {
        indexerMotor.setVoltage(indexerSpeed);
    }

    public void stopIndexer() {
        indexerMotor.setVoltage(0);
    }

    public void reverseIndexer() {
        indexerMotor.setVoltage(-3);
    }
}
