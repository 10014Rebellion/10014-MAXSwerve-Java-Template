package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class commandDrivetrainAimAtSpeaker extends Command {

    private DriveSubsystem swerveDrive;
    private Vision centralCamera;
    private CommandXboxController driverController;

    private PIDController turnController;
    private double calculatedTurnSpeed;
    private double goalYaw;

    public commandDrivetrainAimAtSpeaker(DriveSubsystem swerveDrive, Vision centralCamera, CommandXboxController driverController) {
        this.swerveDrive = swerveDrive;
        this.centralCamera = centralCamera;
        this.driverController = driverController;
        
        addRequirements(swerveDrive, centralCamera);
    }

    @Override
    public void initialize() {
        DriveConstants.currentDriveState = DriveConstants.driveState.AIMING;
    }

    @Override
    public void execute() {
        
 //       SmartDashboard.putNumber("Current Yaw Offset", driverController.get)
    }

    @Override
    public void end(boolean interrupted) {
        DriveConstants.currentDriveState = DriveConstants.driveState.DRIVING;
    }
    @Override
    public boolean isFinished() {
        return false;
    }


}
