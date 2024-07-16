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

    private double flywheelVelocityReference;
    private double flywheelVelocityOffset;

    private TunableNumber flywheelVelocityTunableNumber, flywheelVelocityOffsetTunableNumber;
    private TunableNumber flywheelP, flywheelV, flywheelD;

    public doubleShooterFlywheels() {
        leftFlywheelMotor = new CANSparkFlex(ShooterConstants.kLeftFlywheelMotorCanID, MotorType.kBrushless);
        rightFlywheelMotor = new CANSparkFlex(ShooterConstants.kRightFlywheelMotorCanID, MotorType.kBrushless);

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
        flywheelVelocityReference = 0;
        flywheelVelocityOffset = 0;

        flywheelVelocityTunableNumber = new TunableNumber("Tunable Flywheel Velocity");
        flywheelVelocityOffsetTunableNumber = new TunableNumber("Tunable Flywheel Velocity Offset");
        flywheelP = new TunableNumber("Flywheel P");
        flywheelV = new TunableNumber("Flywheel FF");
        flywheelD = new TunableNumber("Flywheel D");

        flywheelVelocityTunableNumber.setDefault(0);
        flywheelVelocityOffsetTunableNumber.setDefault(0);
        flywheelD.setDefault(0);
        flywheelP.setDefault(0);
        flywheelV.setDefault(0);
    }

    public void setFlywheelVelocity(double targetVelocity) {
        flywheelVelocityReference = targetVelocity;
        double offsetVelocityReference = flywheelVelocityReference * flywheelVelocityOffset;
        leftFlywheelController.setReference(targetVelocity, ControlType.kVelocity);
        rightFlywheelController.setReference(offsetVelocityReference, ControlType.kVelocity);
    }

    public void setVelocityOffset(double offsetPercent) {
        flywheelVelocityOffset = offsetPercent;
    }

    public boolean flywheelAtSetpoint() {
        double offsetVelocityReference = flywheelVelocityReference * flywheelVelocityOffset;
        return 
        (Math.abs(flywheelVelocityReference - leftFlywheelEncoder.getVelocity()) < 500
         && flywheelVelocityReference != 0) &&
        (Math.abs(offsetVelocityReference - rightFlywheelEncoder.getVelocity()) < 500
         && offsetVelocityReference != 0);
    }

    public double getLeftFlywheelVelocity() {
        return leftFlywheelEncoder.getVelocity();
    }

    public double getRightFlywheelVelocity() {
        return rightFlywheelEncoder.getVelocity();
    }

    public void tuneFlywheelVelocity() {
        setFlywheelVelocity(flywheelVelocityTunableNumber.get());
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

        if (flywheelVelocityTunableNumber.hasChanged()) {
            tuneFlywheelVelocity();
        }

        if(flywheelVelocityOffsetTunableNumber.hasChanged()) {
            tuneFlywheelVelocityOffset();
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