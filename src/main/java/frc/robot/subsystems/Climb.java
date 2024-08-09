package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase{
    private final CANSparkMax leftClimbMotor;
    private final CANSparkMax rightClimbMotor;
    //private final CANSparkMax rollerClimbMotor;

    private final AbsoluteEncoder leftClimbEncoder;
    private final AbsoluteEncoder rightClimbEncoder;

    //private final PIDController leftClimbController;
    //private final PIDController rightClimbController;

    public Climb() {

        // climb motor instantiations
        leftClimbMotor = new CANSparkMax(ClimbConstants.kLeftClimbMotorCanID, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(ClimbConstants.kRightClimbMotorCanID, MotorType.kBrushless);
        //rollerClimbMotor  = new CANSparkMax(ClimbConstants.kClimbRollerMotorCanID, MotorType.kBrushless);

        leftClimbMotor.setIdleMode(IdleMode.kBrake);
        rightClimbMotor.setIdleMode(IdleMode.kBrake);
        //rollerClimbMotor.setIdleMode(IdleMode.kBrake);

        leftClimbMotor.setInverted(true);
        rightClimbMotor.setInverted(false);
        //rollerClimbMotor.setInverted(true);

        leftClimbMotor.setSmartCurrentLimit(ClimbConstants.kLeftClimbMotorCurrentLimit);
        rightClimbMotor.setSmartCurrentLimit(ClimbConstants.kRightClimbMotorCurrentLimit);
        //rollerClimbMotor.setSmartCurrentLimit(ClimbConstants.kClimbRollerMotorCurrentLimit);

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

    /*public void runClimbRoller(double targetVoltage) {
        rollerClimbMotor.setVoltage(targetVoltage);
    }*/

    public void moveClimbSeparate(double leftClimbVoltage, double rightClimbVoltage) {
        leftClimbMotor.setVoltage(-leftClimbVoltage);
        rightClimbMotor.setVoltage(-rightClimbVoltage);
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

    /*public Command runClimbRollerCommand(double targetVoltage) {
        return new InstantCommand(() -> runClimbRoller(targetVoltage));
    }*/
}
