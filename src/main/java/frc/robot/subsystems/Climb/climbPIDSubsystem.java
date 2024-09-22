// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climbPIDSubsystem extends SubsystemBase {
  final CANSparkMax leftClimbMotor;
  final CANSparkMax rightClimbMotor;

  final RelativeEncoder leftClimbEncoder;
  final RelativeEncoder rightClimbEncoder;
  
  public climbPIDSubsystem() {
    leftClimbMotor = new CANSparkMax(ClimbConstants.kLeftClimbMotorCanID, MotorType.kBrushless);
    rightClimbMotor = new CANSparkMax(ClimbConstants.kRightClimbMotorCanID, MotorType.kBrushless);
        
    leftClimbEncoder = leftClimbMotor.getEncoder();
    rightClimbEncoder = rightClimbMotor.getEncoder();


    configMotor(leftClimbMotor, leftClimbEncoder, true);
    configMotor(rightClimbMotor, rightClimbEncoder, false);
  }

  private void configMotor(CANSparkBase motor, RelativeEncoder encoder, boolean isInverted) {
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(isInverted);
    motor.setSmartCurrentLimit(ClimbConstants.kClimbCurrentLimit);

    motor.burnFlash();
  }

  public void resetEncoders() {
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);
  }

  public double getLeftEncoder(){
    return leftClimbEncoder.getPosition();
  }

  public double getRightEncoder(){
    return rightClimbEncoder.getPosition();
  }

  public double getAvgEncoder() {
    return (getLeftEncoder() + getRightEncoder()) / 2;
  }

  public void setVoltageOutput(double voltage) {
    setVoltageOutputLeft(voltage);
    setVoltageOutputRight(voltage);
  }

  public void setPercentOutput(double output) {
    setPercentOutputLeft(output);
    setPercentOutputRight(output);
  }

  private boolean atLimit(double output, double encoderReading) {
    return (output < 0 && encoderReading <= ClimbConstants.kEncoderLimitBottom) || (output > 0 && encoderReading >= ClimbConstants.kEncoderLimitTop);
  }

  public void setPercentOutputLeft(double output) {
    double clampedOutput = MathUtil.clamp(output, -1, 1);

    if (!atLimit(clampedOutput, getLeftEncoder()))
      leftClimbMotor.set(clampedOutput);
  }

  public void setPercentOutputRight(double output) {
    double clampedOutput = MathUtil.clamp(output, -1, 1);

    if (!atLimit(clampedOutput, getRightEncoder()))
      rightClimbMotor.set(clampedOutput);
  }

  public void setVoltageOutputLeft(double voltage) {
    double clampedOutput = MathUtil.clamp(voltage, -12.0, 12.0);

    if (!atLimit(clampedOutput, getLeftEncoder()))
      leftClimbMotor.setVoltage(clampedOutput);
  }

  public void setVoltageOutputRight(double voltage) {
    double clampedOutput = MathUtil.clamp(voltage, -12.0, 12.0);

    if (!atLimit(clampedOutput, getRightEncoder()))
      rightClimbMotor.setVoltage(clampedOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Average Encoder", getAvgEncoder());
    SmartDashboard.putNumber("Right Encoder", getRightEncoder());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
  }
}
