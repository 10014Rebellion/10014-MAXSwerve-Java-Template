package frc.robot.subsystems.Shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants.PivotPIDConstants;
import frc.robot.Constants.ShooterConstants;

public class armPIDController extends PIDSubsystem{

    private final CANSparkMax pivotMotor;
    private final AbsoluteEncoder pivotEncoder;
    private final ArmFeedforward armFFControl;

    //Pulls PID constants into more useable forms
    private static final double kP = PivotPIDConstants.kP;
    private static final double kI = PivotPIDConstants.kI;
    private static final double kD = PivotPIDConstants.kD;

    // Pulls FeedForward constants into more useable forms
    private static final double kS = PivotPIDConstants.kS;
    private static final double kG = PivotPIDConstants.kG;
    private static final double kV = PivotPIDConstants.kV;
    private static final double kA = PivotPIDConstants.kA;


    public armPIDController() {

        // Initializes the PIDController.
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(1, 10);
        setSetpoint(PivotPIDConstants.baseSetpoint);

        //Initializes the Feedforward controller
        armFFControl = new ArmFeedforward(kS, kG, kV, kA);

        // Initializes the SparkMAX
        pivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorCanID, MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotMotorCurrentLimit);

        // Initializes the Encoder
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(360);
        pivotEncoder.setInverted(true);

        // Disables the PID to start in order to avoid any weird accidental movement.
        disable();

        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
    }

    @Override
    public double getMeasurement() {
        return (pivotEncoder.getPosition());
    }

    @Override
    public void useOutput(double outputVoltage, double setpoint) {
        // Checks if the arm is physically able to go up / down depending on the 
        // given output. If it isn't, sets to zero (aka no moving)
        // If it is able, then sets the output to that value.
        if (
            (pivotEncoder.getPosition() < ShooterConstants.kArmLowerLimit && outputVoltage < 0) ||
            (pivotEncoder.getPosition() > ShooterConstants.kArmUpperLimit && outputVoltage > 0)
        ) {pivotMotor.setVoltage(0); 
            System.out.println("OUTSIDE BOUNDS"); }
        else {pivotMotor.setVoltage(outputVoltage + armFFControl.calculate(50 * Math.PI/180, 1));}
            //System.out.println("INSIDE BOUNDS, MOVING MOTOR");

        // Outputting the debug data to SmartDashboard
        SmartDashboard.putNumber("PID output", outputVoltage);
        SmartDashboard.putNumber("PID Setpoint", setpoint);
        SmartDashboard.putNumber("Encoder Reading", pivotEncoder.getPosition());
        SmartDashboard.putBoolean("At Setpoint", m_controller.atSetpoint());
        SmartDashboard.putNumber("FF Output", armFFControl.calculate(setpoint * Math.PI/180, 0.1));

        //double tempkP = SmartDashboard.getNumber("kP", 0);
        //System.out.println(tempkP);
    }

    // Sets the setpoint for the PID Controller and runs the loop.
    public void goToSetpoint(double setpoint) {
        setSetpoint(setpoint);
        enable();
    }

    public void editPIDVals() {
        double kP = SmartDashboard.getNumber("kP", 0);
        double kI = SmartDashboard.getNumber("kI", 0);
        double kD = SmartDashboard.getNumber("kD", 0);

        getController().setPID(kP, kI, kD);
    }
}
