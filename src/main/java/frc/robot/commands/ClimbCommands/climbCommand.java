// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Climb.climbPIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climbCommand extends PIDCommand {
  /** Creates a new climbCommand. */
  public climbCommand(climbPIDSubsystem climbSubsystem, double climbGoal, boolean isRightClimb) {
    super(
        // The controller that the command will use
        new PIDController(0.004, 0, 0),
        // This should return the measurement
        () -> (isRightClimb) ? climbSubsystem.getRightEncoder() : climbSubsystem.getLeftEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> climbGoal,
        // This uses the output
        output -> {
          if (isRightClimb)
            climbSubsystem.setPercentOutputRight(
              climbGoal > climbSubsystem.getRightEncoder() ? output : output * 0.25
            );
          else
            climbSubsystem.setPercentOutputLeft(
              climbGoal > climbSubsystem.getLeftEncoder() ? output : output * 0.25
            );
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
