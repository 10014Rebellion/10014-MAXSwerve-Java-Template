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

  public climbBangBangCommand(climbPIDSubsystem climbSubsystem, double climbGoal, boolean isRightClimb) {
    this.mSetpoint = climbGoal;
    this.mIsRight = isRightClimb;
    this.mClimbSubsystem = climbSubsystem;
    // addRequirements(climbSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bangController = new BangBangController(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measurement = (mIsRight) ? mClimbSubsystem.getRightEncoder() : mClimbSubsystem.getLeftEncoder();
    double calculation = bangController.calculate(measurement, mSetpoint);

    if (mIsRight)
      mClimbSubsystem.setPercentOutputRight(calculation);
    else
      mClimbSubsystem.setPercentOutputLeft(calculation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
