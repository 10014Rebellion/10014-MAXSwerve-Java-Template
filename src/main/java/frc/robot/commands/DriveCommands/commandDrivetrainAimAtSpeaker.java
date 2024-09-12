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
        turnController = new PIDController(DriveConstants.kAngularP, 0, DriveConstants.kAngularD);
        addRequirements(swerveDrive, centralCamera);
    }

    @Override
    public void initialize() {
        DriveConstants.currentDriveState = DriveConstants.driveState.AIMING;
    }

    @Override
    public void execute() {
        double goalYaw = centralCamera.getYaw();
        calculatedTurnSpeed = turnController.calculate(centralCamera.getYaw(), 0);
        swerveDrive.drive(
                    -MathUtil.applyDeadband(driverController.getLeftY()*OIConstants.kDriveMult, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(driverController.getLeftX()*OIConstants.kDriveMult, OIConstants.kDriveDeadband),
                    calculatedTurnSpeed, true, false);
        //System.out.println("Camera Yaw: " + centralCamera.getYaw());
        //System.out.println("Calculated output: " + calculatedTurnSpeed);
        DriveConstants.aimedAtTarget = (Math.abs(calculatedTurnSpeed) < 0.1);
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
