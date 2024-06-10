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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotPIDConstants;
import frc.robot.Constants.ShooterConstants;
//import frc.robot.Constants.ShooterConstants.FlywheelPIDConstants;
//import frc.robot.Constants.ShooterConstants.PivotPIDConstants;


public class Shooter extends SubsystemBase {
    
    private final CANSparkMax indexerMotor;
    private final CANSparkMax flywheelMotor;
    private final CANSparkMax pivotMotor;

    private final AbsoluteEncoder pivotEncoder;

    private final PIDController pivotController;

    public Shooter() {

        indexerMotor = new CANSparkMax(ShooterConstants.kIndexerMotorCanID, MotorType.kBrushless);
        flywheelMotor = new CANSparkMax(ShooterConstants.kFlywheelMotorCanID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorCanID, MotorType.kBrushless);

        //indexerMotor.restoreFactoryDefaults(true);
        //flywheelMotor.restoreFactoryDefaults(true);
        //pivotMotor.restoreFactoryDefaults(true);

        indexerMotor.setIdleMode(IdleMode.kCoast);
        flywheelMotor.setIdleMode(IdleMode.kCoast);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        indexerMotor.setInverted(false);
        flywheelMotor.setInverted(false);
        pivotMotor.setInverted(false);

        indexerMotor.setSmartCurrentLimit(ShooterConstants.kIndexerMotorCurrentLimit);
        flywheelMotor.setSmartCurrentLimit(ShooterConstants.kFlywheelMotorCurrentLimit);
        pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotMotorCurrentLimit);

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        pivotController = new PIDController(PivotPIDConstants.kP, PivotPIDConstants.kI, PivotPIDConstants.kD);
    }

    // please dont run permanently when i press it once...
    public void moveArmManually(double targetVoltage) {
        pivotMotor.setVoltage(targetVoltage);
    }

    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    public void runIndexer(double targetVoltage) {
        indexerMotor.setVoltage(targetVoltage);
    }

    public void runFlywheel(double targetVoltage) {
        flywheelMotor.setVoltage(targetVoltage);
    }

    public Command manualMoveArmCommand(double targetVoltage) {
        return new InstantCommand(() -> moveArmManually(targetVoltage));
    }

    public Command runIndexerCommand(double targetVoltage) {
        return new InstantCommand(() -> runIndexer(targetVoltage));
    }

    public Command runFlywheelCommand(double targetVoltage) {
        return new InstantCommand(() -> runFlywheel(targetVoltage));
    }
 }
