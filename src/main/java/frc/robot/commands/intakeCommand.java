package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;

public class intakeCommand extends Command {
    private final CANSparkMax intakeMotor;
    private double intakeSpeed;
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private BooleanTopic noteDetectedTopic = inst.getBooleanTopic("Note Detected");
    private BooleanTopic atSetpointTopic = inst.getBooleanTopic("At Setpoint");
    private final BooleanSubscriber noteDetectionSubscriber;
    private final BooleanSubscriber atSetpointSubscriber;

    public intakeCommand() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
        intakeSpeed = 8;
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        noteDetectionSubscriber = noteDetectedTopic.subscribe(false);
        atSetpointSubscriber = atSetpointTopic.subscribe(false);
    }

    public void setIntakeSpeed(double outputVoltage ){
        intakeSpeed = outputVoltage;
    }

    @Override
    public void initialize() {
            intakeMotor.setVoltage(0);
      // . else indexerMotor.setVoltage(0);
        
    }

    @Override
    public void execute() {
        if(atSetpointSubscriber.get()) {
            intakeMotor.setVoltage(intakeSpeed);
        }
        else intakeMotor.setVoltage(0);
    }

    @Override
    public void end(boolean interrupted) {
        intakeMotor.setVoltage(0);
    }
    
    @Override
    public boolean isFinished() {
        return noteDetectionSubscriber.get();
    }

    public Command forceRunIntake(double outputVoltage) {
        return new InstantCommand(() -> intakeMotor.setVoltage(outputVoltage));
    }
}
