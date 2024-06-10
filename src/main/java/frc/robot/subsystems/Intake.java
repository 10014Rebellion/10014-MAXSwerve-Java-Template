package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
        
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
    }

    public void runIntake(double targetVoltage) {
        intakeMotor.setVoltage(targetVoltage);
    }

    public Command runIntakeCommand(double targetVoltage) {
        return new InstantCommand(() -> runIntake(targetVoltage));
    }
}
