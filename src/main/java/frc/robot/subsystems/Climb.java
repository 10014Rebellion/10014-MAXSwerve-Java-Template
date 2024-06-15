package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
/*import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;*/
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
/*import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotPIDConstants;
import frc.robot.Constants.ShooterConstants; */
import frc.robot.Constants.ClimbConstants;

public class Climb {
    private final CANSparkMax leftClimbMotor;
    private final CANSparkMax rightClimbMotor;

    private final AbsoluteEncoder leftClimbEncoder;
    private final AbsoluteEncoder rightClimbEncoder;

    //private final PIDController leftClimbController;
    //private final PIDController rightClimbController;

    public Climb() {

        // climb motor instantiations
        leftClimbMotor = new CANSparkMax(ClimbConstants.kLeftClimbMotorCanID, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(ClimbConstants.kRightClimbMotorCanID, MotorType.kBrushless);

        leftClimbMotor.setIdleMode(IdleMode.kBrake);
        rightClimbMotor.setIdleMode(IdleMode.kBrake);

        leftClimbMotor.setInverted(false);
        rightClimbMotor.setInverted(true);

        leftClimbMotor.setSmartCurrentLimit(ClimbConstants.kLeftClimbMotorCurrentLimit);
        rightClimbMotor.setSmartCurrentLimit(ClimbConstants.kRightClimbMotorCurrentLimit);

        leftClimbEncoder = leftClimbMotor.getAbsoluteEncoder(Type.kDutyCycle);
        leftClimbEncoder.setPositionConversionFactor(360);

        rightClimbEncoder = rightClimbMotor.getAbsoluteEncoder(Type.kDutyCycle);
        rightClimbEncoder.setPositionConversionFactor(360);

    }

    public void moveLeftClimb(double targetVoltage) {
        leftClimbMotor.setVoltage(targetVoltage);
    }

    public void moveRightClimb(double targetVoltage) {
        rightClimbMotor.setVoltage(targetVoltage);
    }

    public void moveBothClimb(double targetVoltage) {
        leftClimbMotor.setVoltage(targetVoltage);
        rightClimbMotor.setVoltage(targetVoltage);
    }

    public Command moveLeftClimbCommand(double targetVoltage) {
        return new InstantCommand(() -> moveLeftClimb(targetVoltage));
    }

    public Command moveRightClimbCommand(double targetVoltage) {
        return new InstantCommand(() -> moveRightClimb(targetVoltage));
    }

    public Command moveBothClimbCommand(double targetVoltage) {
        return new InstantCommand(() -> moveBothClimb(targetVoltage));
    }
}
