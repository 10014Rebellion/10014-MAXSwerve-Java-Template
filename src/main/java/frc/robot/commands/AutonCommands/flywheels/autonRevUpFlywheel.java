// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;

public class autonRevUpFlywheel extends Command {
  private final doubleShooterFlywheels flywheels;

    public autonRevUpFlywheel (doubleShooterFlywheels f) {
        flywheels = f;
        addRequirements(flywheels);
    }

    @Override
    public void initialize() {
        if (IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(5800, 2900);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.SHOOT;
        }
        else {
            flywheels.setBothFlywheelVelocity(0, 0);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.NOTHING;
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (FlywheelConstants.currentFlywheelState == FlywheelConstants.flywheelState.SHOOT) {
            flywheels.setBothFlywheelVelocity(5800, 2900);
        }
        else if (IndexerConstants.robotHasNote) {
            flywheels.setBothFlywheelVelocity(1000, 1000);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.IDLE;
        }
        else {
            flywheels.setBothFlywheelVelocity(0, 0);
            FlywheelConstants.currentFlywheelState = FlywheelConstants.flywheelState.NOTHING;
        }
    }

    @Override
    public boolean isFinished() {
        return FlywheelConstants.flywheelsAtSetpoint;  
    }
}
