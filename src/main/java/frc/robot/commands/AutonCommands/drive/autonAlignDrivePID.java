package frc.robot.commands.AutonCommands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class autonAlignDrivePID extends Command{

    private DriveSubsystem robotDrive;
    //private CommandXboxController driverController;
    private Supplier<Pose2d> robotPose;
    private Supplier<Double> targetYaw;
    private PIDController turnController;
    /*public Command alignToTargetCommand(Supplier<Pose2d> robotPose, Supplier<Double> targetYaw) {
    return
    run(() -> {
      double robotYaw = robotPose.get().getRotation().getDegrees();
      double speedRotation = turnController.calculate(robotYaw);
      speedRotation += Math.copySign(0.15, speedRotation);
      if (turnController.atSetpoint()) {
        speedRotation = 0.0;
        Constants.DriveConstants.aimedAtTarget = true;
      }
      //setSwerveModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, speedRotation)));
    })
    .beforeStarting(() -> {
      Constants.DriveConstants.aimedAtTarget = false;
      turnController.setSetpoint(targetYaw.get());
      turnController.reset();
    });
  }*/
    public autonAlignDrivePID(DriveSubsystem robotDrive,  Supplier<Pose2d> robotPose, Supplier<Double> targetYaw) {
        this.robotDrive = robotDrive;
        //this.driverController = driverController;
        this.robotPose = robotPose;
        this.targetYaw = targetYaw;
        turnController = new PIDController(DriveConstants.kAngularP, 0, DriveConstants.kAngularD);
        turnController.enableContinuousInput(-180.0, 180.0);
        turnController.setTolerance(1.0);
        addRequirements(robotDrive);
    }

    @Override
    public void initialize() {
        DriveConstants.currentDriveState = DriveConstants.driveState.AIMING;
        Constants.DriveConstants.aimedAtTarget = false;
        turnController.setSetpoint(targetYaw.get());
        turnController.reset();
    }

    @Override
    public void execute() {
        double robotYaw = robotPose.get().getRotation().getDegrees();
        turnController.setSetpoint(targetYaw.get());
        turnController.reset();
        double speedRotation = turnController.calculate(robotYaw);
        //speedRotation += Math.copySign(0.15, speedRotation);
        if (turnController.atSetpoint()) {
            //speedRotation = 0.0;
            Constants.DriveConstants.aimedAtTarget = true;
            DriveConstants.currentDriveState = DriveConstants.driveState.AIMED;
        }
        System.out.println(speedRotation);
        robotDrive.drive(
                    0,
                    0,
                    speedRotation, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        
        DriveConstants.currentDriveState = DriveConstants.driveState.DRIVING;
    }

    @Override
    public boolean isFinished() {
        return Constants.DriveConstants.aimedAtTarget;
    }
}
