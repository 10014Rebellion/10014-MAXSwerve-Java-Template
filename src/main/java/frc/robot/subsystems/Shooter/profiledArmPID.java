package frc.robot.subsystems.Shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.PivotPIDConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class profiledArmPID extends ProfiledPIDSubsystem{

    // Pivot Objects
    private final CANSparkMax pivotMotor;
    private final AbsoluteEncoder pivotEncoder;
    private final ArmFeedforward armFFControl;

    // Shooter Object=s
    private final CANSparkMax flywheelMotor;

    // REV PDH for the switchable channel.

    //Pulls PID constants into more useable forms
    private static final double kP = PivotPIDConstants.kP;
    private static final double kI = PivotPIDConstants.kI;
    private static final double kD = PivotPIDConstants.kD;

    // Pulls FeedForward constants into more useable forms
    private static final double kS = PivotPIDConstants.kS;
    private static final double kG = PivotPIDConstants.kG;
    private static final double kV = PivotPIDConstants.kV;
    private static final double kA = PivotPIDConstants.kA;
    
    // Initializes the network table... Stuff
    // Imma be honest idk how this works yet
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public BooleanTopic noteDetectedTopic = inst.getBooleanTopic("Note Detected");
    public BooleanTopic atSetpointTopic = inst.getBooleanTopic("At Setpoint");
    private final BooleanEntry noteDetectionEntry;
    private final BooleanEntry atSetpointEntry;

    public DigitalInput noteDetector = new DigitalInput(0);
    public DigitalInput noteDetector2 = new DigitalInput(1);
    public PowerDistribution pdh;
    public double setpoint = ShooterConstants.kArmParallelPosition;


    public profiledArmPID() {

        // Initializes the PIDController.
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(PivotPIDConstants.maxVelocity, PivotPIDConstants.maxAccel)));
        getController().setTolerance(PivotPIDConstants.errorLimit, 10);
        setGoal(ShooterConstants.kArmParallelPosition);
        
        //Establishes the entry.
        noteDetectionEntry = noteDetectedTopic.getEntry(false);
        atSetpointEntry = atSetpointTopic.getEntry(false);
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

        // Initializes the Flywheel and Indexer motors

        flywheelMotor = new CANSparkMax(38, MotorType.kBrushless);
        flywheelMotor.setIdleMode(IdleMode.kCoast);
        flywheelMotor.setInverted(false);
        flywheelMotor.setSmartCurrentLimit(ShooterConstants.kFlywheelMotorCurrentLimit);

        pdh = new PowerDistribution(1, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        // Disables the PID to start in order to avoid any weird accidental movement.
        m_controller.reset(pivotEncoder.getPosition());
        disable();

        SmartDashboard.putNumber("kP", PivotPIDConstants.kP);
        
    }

    @Override
    public double getMeasurement() {
        return (pivotEncoder.getPosition());
    }

    @Override
    public void useOutput(double outputVoltage, TrapezoidProfile.State state) {

        // Calculates the output of the feedforward controller
        double FFOutput = armFFControl.calculate(pivotEncoder.getPosition() * Math.PI / 180, 10);

        double totalOutput = outputVoltage + FFOutput;
        // Runs a check that the controller isn't trying to go outside the bounds for whatever reason.
        if (!atSetpoint()) {
            if (((pivotEncoder.getPosition() < ShooterConstants.kArmLowerLimit) && (totalOutput < 0) ||
                ((pivotEncoder.getPosition() > ShooterConstants.kArmUpperLimit) && (totalOutput > 0))))
                {pivotMotor.setVoltage(0); 
                System.out.println("OUTSIDE BOUNDS");}
            // Then, it lets it move.
            else if (totalOutput == FFOutput) {
                pivotMotor.setVoltage(0);
                System.out.println("Just FF Running?");
            }
            else {
                if(totalOutput > 12) {
                    pivotMotor.setVoltage(12);
                }
                else if (totalOutput < -12) {
                    pivotMotor.setVoltage(-12);
                }
                else {
                    pivotMotor.setVoltage(totalOutput);
                }
                
            }
        }
        
        noteDetectionEntry.set((!noteDetector.get()) || (!noteDetector2.get()));
        atSetpointEntry.set(atSetpoint());
        SmartDashboard.putNumber("PID+FF output", totalOutput);
        SmartDashboard.putNumber("PID Output", outputVoltage);
        SmartDashboard.putNumber("FF Output", FFOutput);
        SmartDashboard.putNumber("Arm Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("PID Setpoint", setpoint);
        SmartDashboard.putNumber("Pivot Current", pivotMotor.getOutputCurrent());
        pdh.setSwitchableChannel(noteDetectionEntry.get());
    }

    public void goToSetpoint(double setpoint) {
        setGoal(setpoint);
        this.setpoint = setpoint;
        enable();
    }

    public Command goToSetpointCommand(double setpoint) {
        return new InstantCommand(() ->
        goToSetpoint(setpoint));
        
    }

    public void runArmFFOnly() {
        disable();
        double FFOutput = armFFControl.calculate(ShooterConstants.kArmParallelPosition * Math.PI / 180, 10);
        pivotMotor.setVoltage(FFOutput);
    }

    public void runManualArmCommand(double outputVoltage) {
        disable();
        pivotMotor.setVoltage(outputVoltage);
    }
    public void stopRunningArm() {
        disable();
        pivotMotor.setVoltage(0);
    }

    public Command runFlywheelCommand(double targetVoltage) {
        return new InstantCommand(() -> 
        flywheelMotor.setVoltage(targetVoltage));
    }
    
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void runFFOnlyCommand() {
        double FFOutput = armFFControl.calculate(ShooterConstants.kArmParallelPosition * Math.PI / 180, 10);
        disable();
        pivotMotor.setVoltage(FFOutput);
    }
}
