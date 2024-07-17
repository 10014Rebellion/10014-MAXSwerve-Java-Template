package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class indexerSubsystem extends SubsystemBase{

    public CANSparkMax indexerMotor;
    public PowerDistribution pdh;
    public DigitalInput noteDetector1 = new DigitalInput(IndexerConstants.kNoteDetector1Port);
    public DigitalInput noteDetector2 = new DigitalInput(IndexerConstants.kNoteDetector2Port);
    public double indexerSpeed = 1.5;

    public indexerSubsystem() {
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerMotorCanID, MotorType.kBrushless);
        indexerMotor.setIdleMode(IdleMode.kCoast);
        indexerMotor.setInverted(true);
        indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerMotorCurrentLimit);

        pdh = new PowerDistribution(1, ModuleType.kRev);
        pdh.setSwitchableChannel(false);
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

    @Override
    public void periodic() {
        IndexerConstants.robotHasNote = !noteDetector1.get() || !noteDetector2.get();
        pdh.setSwitchableChannel(IndexerConstants.robotHasNote);
        SmartDashboard.putBoolean("Note Detected", IndexerConstants.robotHasNote);
    }
}
