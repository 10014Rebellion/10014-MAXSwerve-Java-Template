package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.utils.TunableNumber;
import frc.robot.Constants.ShooterConstants;

public class doubleShooterFlywheels extends SubsystemBase{ 

    private CANSparkFlex leftFlywheelMotor;
    private CANSparkFlex rightFlywheelMotor;

    private RelativeEncoder leftFlywheelEncoder;
    private RelativeEncoder rightFlywheelEncoder;

    private SparkPIDController leftFlywheelController;
    private SparkPIDController rightFlywheelController;

    private double leftFlywheelVelocityReference;
    private double rightFlywheelVelocityReference;
    private double flywheelVelocityOffset;

    private TunableNumber leftFlywheelVelocityTunableNumber, rightFlywheelVelocityTunableNumber;
    private TunableNumber flywheelP, flywheelV, flywheelD;

    public doubleShooterFlywheels() {
        leftFlywheelMotor = new CANSparkFlex(ShooterConstants.kLeftFlywheelMotorCanID, MotorType.kBrushless);
        rightFlywheelMotor = new CANSparkFlex(ShooterConstants.kRightFlywheelMotorCanID, MotorType.kBrushless);

        leftFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kLeftFlywheelMotorCurrentLimit);
        rightFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kRightFlywheelMotorCurrentLimit);

        leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
        rightFlywheelMotor.setIdleMode(IdleMode.kCoast);

        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);

        leftFlywheelEncoder = leftFlywheelMotor.getEncoder();
        rightFlywheelEncoder = rightFlywheelMotor.getEncoder();

        // These 2 controllers will have identical P, V, and D values, but different set velocities.
        // So, they must be different objects.
        leftFlywheelController = leftFlywheelMotor.getPIDController();
        rightFlywheelController = rightFlywheelMotor.getPIDController();
        leftFlywheelVelocityReference = 0;
        rightFlywheelVelocityReference = 0;
        flywheelVelocityOffset = 0;

        leftFlywheelVelocityTunableNumber = new TunableNumber("Tunable Left Flywheel Velocity");
        rightFlywheelVelocityTunableNumber = new TunableNumber("Tunable Right Flywheel Velocity");
        flywheelP = new TunableNumber("Flywheel P");
        flywheelV = new TunableNumber("Flywheel V");
        flywheelD = new TunableNumber("Flywheel D");

        leftFlywheelVelocityTunableNumber.setDefault(0);
        rightFlywheelVelocityTunableNumber.setDefault(0);
        flywheelD.setDefault(0.0001);
        flywheelP.setDefault( 0.0003);
        flywheelV.setDefault(0.00017);
        // Good D: 0.0001
        // Good P: 0.0003
        // Good V: 0.00017
    }

    public void setLeftFlywheelVelocity(double targetVelocity) {
        leftFlywheelVelocityReference = targetVelocity;
        leftFlywheelController.setReference(targetVelocity, ControlType.kVelocity);
    }

    public void setRightFlywheelVelocity(double targetVelocity) {
        rightFlywheelVelocityReference = targetVelocity;
        rightFlywheelController.setReference(targetVelocity, ControlType.kVelocity);
        
    }

    public void setVelocityOffset(double offsetPercent) {
        flywheelVelocityOffset = offsetPercent;
    }

    public boolean flywheelAtSetpoint() {
        //double offsetVelocityReference = flywheelVelocityReference * flywheelVelocityOffset;
        return 
        (Math.abs(leftFlywheelVelocityReference - leftFlywheelEncoder.getVelocity()) < 500
         && leftFlywheelVelocityReference != 0) &&
        (Math.abs(rightFlywheelVelocityReference - rightFlywheelEncoder.getVelocity()) < 500
         && rightFlywheelVelocityReference != 0);
    }

    public double getLeftFlywheelVelocity() {
        return leftFlywheelEncoder.getVelocity();
    }

    public double getRightFlywheelVelocity() {
        return rightFlywheelEncoder.getVelocity();
    }

    public void tuneLeftFlywheelVelocity() {
        setLeftFlywheelVelocity(leftFlywheelVelocityTunableNumber.get());
    }

    public void tuneRightFlywheelVelocity() {
        setRightFlywheelVelocity(rightFlywheelVelocityTunableNumber.get());
    }

    public void tuneFlywheelVelocityOffset() {
        setVelocityOffset(flywheelVelocityOffset);
    }

    public void tuneFlywheelP() {
        leftFlywheelController.setP(flywheelP.get());
        rightFlywheelController.setP(flywheelP.get());
    }

    private void tuneFlywheelV() {
        leftFlywheelController.setFF(flywheelV.get());
        rightFlywheelController.setFF(flywheelV.get());
    }

    private void tuneFlywheelD() {
        leftFlywheelController.setD(flywheelD.get());
        rightFlywheelController.setD(flywheelD.get());
    }
    
    public void periodic() {

        SmartDashboard.putNumber("Current Left Flywheel Velocity", getLeftFlywheelVelocity());
        SmartDashboard.putNumber("Current Right Flywheel Velocity", getRightFlywheelVelocity());
        SmartDashboard.putNumber("Left Flywheel Current", leftFlywheelMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Flywheel Current", rightFlywheelMotor.getOutputCurrent());

        if (leftFlywheelVelocityTunableNumber.hasChanged()) {
            tuneLeftFlywheelVelocity();
        }

        if(rightFlywheelVelocityTunableNumber.hasChanged()) {
            tuneRightFlywheelVelocity();
        }
    
        if (flywheelP.hasChanged()) {
            tuneFlywheelP();
        }
    
        if (flywheelV.hasChanged()) {
            tuneFlywheelV();
        }
    
        if (flywheelD.hasChanged()) {
            tuneFlywheelD();
        }
    }
    // Manual control.
    public void setFlywheelVoltage(double targetVoltage) {
        leftFlywheelMotor.setVoltage(targetVoltage);
        rightFlywheelMotor.setVoltage(targetVoltage);
    }

    public void setFlywheelVoltage(double leftTargetVoltage, double rightTargetVoltage) {
        leftFlywheelMotor.setVoltage(leftTargetVoltage);
        rightFlywheelMotor.setVoltage(rightTargetVoltage);
    }
}