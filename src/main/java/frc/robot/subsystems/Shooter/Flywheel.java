package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Flywheel extends SubsystemBase {
    private CANSparkFlex motor;
    private RelativeEncoder encoder;
    private SparkPIDController controller;

    private double targetVelo;


    public Flywheel(int canID, int currentLimit, boolean inverted) {
        motor = new CANSparkFlex(canID, MotorType.kBrushless);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(inverted);
        encoder = motor.getEncoder();
        controller = motor.getPIDController();
    }

    public void initPID(double P, double D, double FF) {
        targetVelo = 0;
        controller.setP(P);
        controller.setD(D);
        controller.setFF(FF);
    
        controller.setOutputRange(0, 1);
        motor.burnFlash();    
    }

    public void setP(double P) {
        controller.setP(P);
    }

    public void setD(double D) {
        controller.setD(D);
    }

    public void setFF(double FF) {
        controller.setFF(FF);
    }

    public void setTargetVelo(double target) {
        targetVelo = target;
        controller.setReference(targetVelo, ControlType.kVelocity);
    }

    public boolean atSetpoint() {
        return (Math.abs(targetVelo - getVelo()) < 200
                && targetVelo != 0
        );
    }

    public double getVelo() {
        return encoder.getVelocity();
    }

    public void setVoltage(double targetVoltage) {
        motor.setVoltage(targetVoltage);
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    double getAppliedOutput() {
        return motor.getAppliedOutput();
    }
}
