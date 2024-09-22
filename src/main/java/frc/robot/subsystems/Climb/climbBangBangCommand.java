// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;

public class climbBangBangCommand extends Command {

  private final double mSetpoint;
  private final boolean mIsRight;
  private final climbPIDSubsystem mClimbSubsystem;
  private BangBangController bangController;
  private double kTolerance = 5; // Ticks
  private double kSpeedPercent = 75;
  private double kChillRange = 200; // Within 200 ticks, it will be slower

  public climbBangBangCommand(climbPIDSubsystem climbSubsystem, double climbGoal, boolean isRightClimb) {
    this.mSetpoint = climbGoal;
    this.mIsRight = isRightClimb;
    this.mClimbSubsystem = climbSubsystem;
    // addRequirements(climbSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measurement = getMeasurement();
    double currentSpeed = kSpeedPercent / 100;
    
    if(!isFinished()) {

      if(Math.abs(mSetpoint - measurement) < kChillRange)
        currentSpeed *= 0.5; // Half the current speed

      // If error is negative / climb pulling down
      if(mSetpoint - measurement < 0) {
        setOutput(-currentSpeed);

      // If error is positive / climb letting go
      } else {
        setOutput(currentSpeed);
      }
    }
  }

  private double getMeasurement() {
    return (mIsRight) ? mClimbSubsystem.getRightEncoder() : mClimbSubsystem.getLeftEncoder();
  }

  private void setOutput(double output){
    if (mIsRight)
      mClimbSubsystem.setPercentOutputRight(output);
    else
      mClimbSubsystem.setPercentOutputLeft(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(mSetpoint - getMeasurement()) <= kTolerance;
  }
}
