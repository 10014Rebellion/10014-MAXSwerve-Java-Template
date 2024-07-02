package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ShooterConstants;

public class shooterFlywheels extends SubsystemBase{
    
    /*public CANSparkMax flywheelMotor; 
    public double flywheelVoltage = 12;

    public shooterFlywheels() {
        flywheelMotor = new CANSparkMax(ShooterConstants.kFlywheelMotorCanID, MotorType.kBrushless);
        flywheelMotor.setIdleMode(IdleMode.kCoast);
        flywheelMotor.setInverted(false);
        flywheelMotor.setSmartCurrentLimit(ShooterConstants.kFlywheelMotorCurrentLimit);

        flywheelVoltage = 12;
    }

    public void setFlywheelSpeed(double outputVoltage) {
        flywheelVoltage = outputVoltage;
    }

    public Command runFlywheelForward() {
        return new InstantCommand(() -> flywheelMotor.setVoltage(flywheelVoltage));
    }

    public Command runFlywheelBackward() {
        return new InstantCommand(() -> flywheelMotor.setVoltage(-flywheelVoltage));
    }

    public Command dontRunFlywheel() {
        return new InstantCommand(() -> flywheelMotor.setVoltage(0));
    }*/
}
