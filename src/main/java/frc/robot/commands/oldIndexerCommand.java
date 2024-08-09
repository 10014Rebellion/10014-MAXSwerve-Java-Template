package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;


public class oldIndexerCommand extends Command {

    public CANSparkMax indexerMotor;
    //public DigitalInput noteDetector = new DigitalInput(0);
    public double indexerSpeed = 1.5;
    public boolean ignoreDetectionBool = true;
    
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private BooleanTopic noteDetectedTopic = inst.getBooleanTopic("Note Detected");
    public BooleanTopic atSetpointTopic = inst.getBooleanTopic("At Setpoint");
    private final BooleanSubscriber noteDetectionSubscriber;

    public oldIndexerCommand() {
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerMotorCanID, MotorType.kBrushless);
        indexerMotor.setIdleMode(IdleMode.kCoast);
        indexerMotor.setInverted(true);
        indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerMotorCurrentLimit);

        noteDetectionSubscriber = noteDetectedTopic.subscribe(false);
    }

    public void setIndexerSpeed(double outputVoltage ){
        indexerSpeed = outputVoltage;
    }

    public void ignoreDetection(boolean ignore) {
        ignoreDetectionBool = ignore;
    }

    @Override
    public void initialize() {
        setIndexerSpeed(4.75);
        indexerMotor.setVoltage(indexerSpeed);
      // . else indexerMotor.setVoltage(0);
        
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        indexerMotor.setVoltage(0);
    }
    
    @Override
    public boolean isFinished() {
        return noteDetectionSubscriber.get();
        
    }

    public Command forceRunIndexer(double outputVoltage) {
        return new InstantCommand(() -> indexerMotor.setVoltage(outputVoltage));
    }
}


