package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class indexerSubsystem extends SubsystemBase{

    public CANSparkMax indexerMotor;
    public DigitalInput noteDetector1;
    public DigitalInput noteDetector2;
    public double indexerSpeed = 1.5;

    public indexerSubsystem() {
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerMotorCanID, MotorType.kBrushless);
        indexerMotor.setIdleMode(IdleMode.kCoast);
        indexerMotor.setInverted(true);
        indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerMotorCurrentLimit);
        indexerMotor.burnFlash();

        noteDetector1 = new DigitalInput(IndexerConstants.kNoteDetector1Port);
        noteDetector2 = new DigitalInput(IndexerConstants.kNoteDetector2Port);
    }

    public void runIndexer() {
        indexerMotor.setVoltage(1.4);
    }
    public void runIndexerFast() {
        indexerMotor.setVoltage(12);
    }

    public void stopIndexer() {
        indexerMotor.setVoltage(0);
    }

    public void reverseIndexer() {
        indexerMotor.setVoltage(-3);
    }

    public void enableBrake() {
        indexerMotor.setIdleMode(IdleMode.kBrake);
    }
    public void disableBrake() {
        indexerMotor.setIdleMode(IdleMode.kCoast);
    }

    public void getOuttaHere() {
        indexerMotor.setVoltage(-12);
    }
    @Override
    public void periodic() {
        IndexerConstants.robotHasNote = !noteDetector1.get() || !noteDetector2.get();
        SmartDashboard.putBoolean("Note Detected", IndexerConstants.robotHasNote);
        SmartDashboard.putBoolean("Beam break", noteDetector1.get());
    }
}
