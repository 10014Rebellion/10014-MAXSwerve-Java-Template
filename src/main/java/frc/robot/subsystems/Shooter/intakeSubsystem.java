package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class intakeSubsystem extends SubsystemBase{

    private final CANSparkMax intakeMotor;
    private double intakeSpeed = 8;

    public intakeSubsystem() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
    }

    public void runIntake() {
        intakeMotor.setVoltage(intakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.setVoltage(0);
    }

    public void reverseIntake() {
        intakeMotor.setVoltage(-6);
    }
}
