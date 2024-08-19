package frc.robot.subsystems.Shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm {
    private CANSparkMax motor;
    private AbsoluteEncoder encoder;
    private ArmFeedforward FFControl;

    private double kP;
    private double kI;
    private double kD;

    

}
