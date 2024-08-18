package frc.robot.subsystems.Shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.PivotPIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.photonConstants;
import frc.robot.utils.TunableNumber;

public class profiledArmPID extends ProfiledPIDSubsystem{

    // Pivot Objects
    private final CANSparkMax pivotMotor;
    private final AbsoluteEncoder pivotEncoder;
    private final ArmFeedforward armFFControl;

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

    //public DigitalInput noteDetector = new DigitalInput(0);
    //public DigitalInput noteDetector2 = new DigitalInput(1);
    
    public double setpoint = ShooterConstants.kArmParallelPosition;
    public TunableNumber tunableSetpoint;
    public double pivotPos = 0;

    private InterpolatingDoubleTreeMap pivotAngleMap;


    public profiledArmPID() {

        // Initializes the PIDController.
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(PivotPIDConstants.maxVelocity, PivotPIDConstants.maxAccel)));
        getController().setTolerance(PivotPIDConstants.errorLimit, 10);
        

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
        pivotEncoder.setZeroOffset(ShooterConstants.kArmZeroOffset);

        // Converts the encoder readings into a -180 -> 180 deg format.
        if (pivotEncoder.getPosition() > 180) {
            pivotPos = pivotEncoder.getPosition() - 360;
        }
        else pivotPos = pivotEncoder.getPosition();

        // Disables the PID to start in order to avoid any weird accidental movement.
        m_controller.reset(pivotPos);
        disable();

        tunableSetpoint = new TunableNumber("Arm Tunable Setpoint");
        tunableSetpoint.setDefault(0);

        setGoal(ShooterConstants.kArmParallelPosition);
        SmartDashboard.putNumber("kP", PivotPIDConstants.kP);
        pivotAngleMap = new InterpolatingDoubleTreeMap();
        populatePivotAngleMap();
    }

    @Override
    public double getMeasurement() {
        return (pivotPos);
    }

    @Override
    public void useOutput(double outputVoltage, TrapezoidProfile.State state) {

        // Converts the encoder readings into a -180 -> 180 deg format.
        if (pivotEncoder.getPosition() > 180) {
            pivotPos = pivotEncoder.getPosition() - 360;
        }
        else pivotPos = pivotEncoder.getPosition();

        // Calculates the output of the feedforward controller
        double FFOutput = armFFControl.calculate(pivotPos * Math.PI / 180, 10);

        double totalOutput = outputVoltage + FFOutput;
        // Runs a check that the controller isn't trying to go outside the bounds for whatever reason.
            if (((pivotPos < ShooterConstants.kArmLowerLimit) && (totalOutput < 0) ||
                ((pivotPos > ShooterConstants.kArmUpperLimit) && (totalOutput > 0))))
                {pivotMotor.setVoltage(0); 
                System.out.println("OUTSIDE BOUNDS");}
            // Then, it lets it move.
            else if (totalOutput == FFOutput) {
                pivotMotor.setVoltage(0);
                System.out.println("Just FF Running?");
            }
            // If the PID is trying to tell it to go beyond the realm of reality,
            // It says no.
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
        
        SmartDashboard.putNumber("PID+FF output", totalOutput);
        SmartDashboard.putNumber("PID Output", outputVoltage);
        SmartDashboard.putNumber("FF Output", FFOutput);
        SmartDashboard.putNumber("Arm Position", pivotPos);
        SmartDashboard.putNumber("PID Setpoint", setpoint);
        SmartDashboard.putNumber("Pivot Current", pivotMotor.getOutputCurrent());
        if (pivotAngleMap != null) {
            SmartDashboard.putNumber("Current Pivot Map Setpoint", pivotAngleMap.get(photonConstants.speakerDistance));
        }
    }

    public void goToSetpoint(double setpoint) {
        
        if (setpoint > ShooterConstants.kArmUpperLimit) {
            setGoal(ShooterConstants.kArmUpperLimit);
            this.setpoint = ShooterConstants.kArmUpperLimit;
        }
        else if (setpoint < ShooterConstants.kArmLowerLimit) {
            setGoal(ShooterConstants.kArmLowerLimit);
            this.setpoint = ShooterConstants.kArmLowerLimit;
        }
        else {
            setGoal(setpoint);
            this.setpoint = setpoint;
        }
        enable();
    }

    public void altGoToSetpoint(double setpoint) {
        if (setpoint > ShooterConstants.kArmUpperLimit) {
            setGoal(ShooterConstants.kArmUpperLimit);
            this.setpoint = ShooterConstants.kArmUpperLimit;
        }
        else if (setpoint < ShooterConstants.kArmLowerLimit) {
            setGoal(ShooterConstants.kArmLowerLimit);
            this.setpoint = ShooterConstants.kArmLowerLimit;
        }
        else {
            setGoal(setpoint);
            this.setpoint = setpoint;
        }
        enable();
    }

    public void goToTunableSetpoint() {
        double tunableSetpointNum = tunableSetpoint.get();
        if (tunableSetpointNum > ShooterConstants.kArmUpperLimit) {
            setGoal(ShooterConstants.kArmUpperLimit);
            this.setpoint = ShooterConstants.kArmUpperLimit;
        }
        else if (tunableSetpointNum < ShooterConstants.kArmLowerLimit) {
            setGoal(ShooterConstants.kArmLowerLimit);
            this.setpoint = ShooterConstants.kArmLowerLimit;
        }
        else {
            setGoal(tunableSetpointNum);
            this.setpoint = tunableSetpointNum;
        }
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
    
    public boolean atSetpoint() {
        return (Math.abs(pivotPos - setpoint) < PivotPIDConstants.errorLimit);
    }

    public double getCalculatedSpeakerAngle() {
        
        if (pivotAngleMap != null) {
            return pivotAngleMap.get(photonConstants.speakerDistance);
        }
        else return ShooterConstants.kArmParallelPosition;
    }

    public void goToCalculatedSpeakerAngle() {
        goToSetpoint(getCalculatedSpeakerAngle());
    }

    public void populatePivotAngleMap() {
        // THESE ARE TEST ANGLES. THEY WILL NOT BE ACCURATE.
        pivotAngleMap.put(1.0, -30.0);
        pivotAngleMap.put(1.5, -20.0);
        pivotAngleMap.put(2.0, -10.0);
        pivotAngleMap.put(2.5, 0.0);
        pivotAngleMap.put(3.0, 10.0);
    }
}
