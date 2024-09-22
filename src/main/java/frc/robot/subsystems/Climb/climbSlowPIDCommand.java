// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climbSlowPIDCommand extends ProfiledPIDCommand {
  /** Creates a new climbProfileCommand. */
  public climbSlowPIDCommand(climbPIDSubsystem climbSubsystem, double climbGoal, boolean isRightClimb) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.002,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1000, 5000)),
        // This should return the measurement
        () -> (isRightClimb) ? climbSubsystem.getRightEncoder() : climbSubsystem.getLeftEncoder(),
        // This should return the goal (can also be a constant)
        () -> climbGoal,
        // This uses the output
        (output, setpoint) -> {
          if (isRightClimb)
            climbSubsystem.setPercentOutputRight(
              climbGoal > climbSubsystem.getRightEncoder() ? output : output 
            );
          else
            climbSubsystem.setPercentOutputLeft(
              climbGoal > climbSubsystem.getLeftEncoder() ? output : output 
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
