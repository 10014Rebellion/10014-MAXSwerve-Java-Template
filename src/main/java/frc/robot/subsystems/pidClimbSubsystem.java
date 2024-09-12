package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utils.TunableNumber;

public class pidClimbSubsystem extends SubsystemBase{
    private final CANSparkMax leftClimbMotor;
    private final CANSparkMax rightClimbMotor;

    private final RelativeEncoder leftClimbEncoder;
    private final RelativeEncoder rightClimbEncoder;

    private PIDController leftPIDController;
    private PIDController rightPIDController;
    public double leftOutput;
    public double rightOutput;

    public TunableNumber tunableSetpoint;
    public double setpoint;
    public boolean runPID;

    public TunableNumber leftClimbP;
    public TunableNumber leftClimbD;

    public TunableNumber rightClimbP;
    public TunableNumber rightClimbD;

    private DigitalInput leftClimbDetector;
    private DigitalInput rightClimbDetector;


    public pidClimbSubsystem() {

        leftClimbMotor = new CANSparkMax(ClimbConstants.kLeftClimbMotorCanID, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(ClimbConstants.kRightClimbMotorCanID, MotorType.kBrushless);
        leftClimbMotor.setIdleMode(IdleMode.kBrake);
        rightClimbMotor.setIdleMode(IdleMode.kBrake);
        leftClimbMotor.setInverted(true);
        rightClimbMotor.setInverted(false);
        leftClimbMotor.setSmartCurrentLimit(ClimbConstants.kLeftClimbMotorCurrentLimit);
        rightClimbMotor.setSmartCurrentLimit(ClimbConstants.kRightClimbMotorCurrentLimit);

        leftClimbEncoder = leftClimbMotor.getAlternateEncoder(8192);
        rightClimbEncoder = rightClimbMotor.getAlternateEncoder(8192);
        leftClimbEncoder.setPositionConversionFactor(2.16);
        rightClimbEncoder.setPositionConversionFactor(2.16);

        leftPIDController = new PIDController(0, 0, 0);
        rightPIDController = new PIDController(0, 0, 0);

        leftClimbP = new TunableNumber("Left Climb P");
        leftClimbD = new TunableNumber("Left Climb D");
        leftClimbP.setDefault(0);
        leftClimbD.setDefault(0);

        rightClimbP = new TunableNumber("Right Climb P");
        rightClimbD = new TunableNumber("Right Climb D");
        rightClimbP.setDefault(0);
        rightClimbD.setDefault(0);

        tunableSetpoint = new TunableNumber("Climb Tunable Setpoint");
        tunableSetpoint.setDefault(0);

        leftClimbDetector = new DigitalInput(ClimbConstants.kLeftClimbDetector);
        rightClimbDetector = new DigitalInput(ClimbConstants.kRightClimbDetector);

        runPID = false;
    }

    public double getLeftClimbPosition() {
        return leftClimbEncoder.getPosition();
    }

    public double getRightClimbPosition() {
        return rightClimbEncoder.getPosition();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void enable() {
        runPID = true;
    }

    public void disable() {
        runPID = false;
    }

    public void moveLeftClimb(double outputPercent) {
        leftClimbMotor.set(outputPercent);
    }

    public void moveRightClimb(double outputPercent) {
        rightClimbMotor.set(outputPercent);
    }

    public void moveBothClimb(double outputVoltage) {
        leftClimbMotor.setVoltage(outputVoltage);
        rightClimbMotor.setVoltage(outputVoltage);
    }

    public boolean getLeftBottomSensor() {
        if (leftClimbDetector != null) {
            return leftClimbDetector.get();
        }
        else return true;
    }

    public boolean getRightBottomSensor() {
        if (rightClimbDetector != null) {
            return rightClimbDetector.get();
        }
        else return true;
    }

    public void resetLeftEncoder() {
        leftClimbEncoder.setPosition(0);
    }
    public void resetRightEncoder() {
        rightClimbEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            leftOutput = leftPIDController.calculate(leftClimbEncoder.getPosition(), setpoint);
            rightOutput = rightPIDController.calculate(rightClimbEncoder.getPosition(), setpoint);

            if(leftClimbP.hasChanged()) {
                leftPIDController.setP(leftClimbP.get());
            }
            if(leftClimbD.hasChanged()) {
                leftPIDController.setD(leftClimbD.get());
            }
            if(rightClimbP.hasChanged()) {
                rightPIDController.setP(rightClimbP.get());
            }
            if(rightClimbD.hasChanged()) {
                rightPIDController.setD(rightClimbD.get());
            }
            if(tunableSetpoint.hasChanged()) {
                setSetpoint(tunableSetpoint.get());
            }
        }
        SmartDashboard.putNumber("Left Climb Position", leftClimbEncoder.getPosition());
        SmartDashboard.putNumber("Right Climb Position", rightClimbEncoder.getPosition());
        SmartDashboard.putNumber("Left PID Output", leftOutput);
        SmartDashboard.putNumber("Right PID Output", rightOutput);
        
    }
}
