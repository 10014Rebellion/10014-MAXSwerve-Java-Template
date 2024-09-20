// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.photonConstants;
import frc.robot.Constants.ShooterConstants.armState;
import frc.robot.subsystems.Shooter.profiledArmPID;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;
import frc.robot.subsystems.LEDInterface;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climb.climbPIDSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.indexerSubsystem;
import frc.robot.subsystems.Shooter.intakeSubsystem;

//import frc.robot.commands.forceIndexCommand;
import frc.robot.commands.IndexerCommands.commandIndexerReverse;
import frc.robot.commands.IndexerCommands.commandIndexerStart;
import frc.robot.commands.ArmCommands.commandArmAlignToTarget;
import frc.robot.commands.ArmCommands.commandArmAmp;
import frc.robot.commands.ArmCommands.commandArmAutoAim;
import frc.robot.commands.ArmCommands.commandArmIntake;
import frc.robot.commands.ArmCommands.commandArmSubwoofer;
// import frc.robot.commands.ClimbCommands.commandClimbAutoZero;
import frc.robot.commands.DriveCommands.commandDrivetrainAimAtSpeaker;
import frc.robot.commands.DriveCommands.commandDrivetrainAlignToTarget;
import frc.robot.commands.IndexerCommands.commandIndexBrakeMode;
import frc.robot.commands.IndexerCommands.commandIndexerPickup;
import frc.robot.commands.IntakeCommands.commandIntakePickup;
import frc.robot.commands.IntakeCommands.commandIntakeStart;
import frc.robot.commands.flywheelCommands.CommandManualFlywheels;
import frc.robot.commands.flywheelCommands.commandFlywheelIdle;
import frc.robot.commands.flywheelCommands.commandFlywheelShoot;
import frc.robot.commands.IntakeCommands.commandIntakeReverse;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.time.Instant;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    //private final profiledArmPID robotShooter = new profiledArmPID();
    //private final Climb          robotClimb = new Climb();
    // Robot Subsystem Creations
    private final DriveSubsystem m_robotDrive;
    private final profiledArmPID robotShooter;
    private final climbPIDSubsystem robotClimb;
    private final doubleShooterFlywheels robotFlywheels;
    private final indexerSubsystem robotIndexer;
    private final intakeSubsystem robotIntake;

    private final Vision centralCamera;
    private final PoseSubsystem poseSubsystem;

    private final LEDInterface robotLED;

    private final Pose2d defaultPose;                                               

    // The driver's controller
    CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    // Copilot's controller
    CommandXboxController copilotController = new CommandXboxController(OIConstants.kCopilotControllerPort);

    // Sets up Auton chooser.
    private final SendableChooser<Command> autoChooser;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        centralCamera = new Vision(photonConstants.kCameraName, photonConstants.kCameraLocation);
        // Robot subsystem initialization.
        m_robotDrive = new DriveSubsystem(centralCamera);
        robotShooter = new profiledArmPID();
        robotClimb = new climbPIDSubsystem();
        robotFlywheels = new doubleShooterFlywheels();
        robotIndexer = new indexerSubsystem();
        robotIntake = new intakeSubsystem();
        robotLED = new LEDInterface();
        defaultPose = new Pose2d(0, 0, new Rotation2d(0));
        poseSubsystem = new PoseSubsystem(centralCamera, m_robotDrive::getRotation2d, m_robotDrive::getSwerveModulePositions);
        
        
        // Configure the button bindings
        
        configureButtonBindings();
        configureTriggers();

        // Auton Sendable Chooser
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Shoot, no move", shootNoMoveAuton());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(driverController.getLeftY()*OIConstants.kDriveMult, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(driverController.getLeftX()*OIConstants.kDriveMult, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(driverController.getRightX()*OIConstants.kTurnMult, OIConstants.kDriveDeadband),
                    true, false),
                m_robotDrive));

        /*robotArmPID.setDefaultCommand(
            robotArmPID.disablePID()
        );*/
        /*robotClimb.setDefaultCommand(
            new InstantCommand(
                () -> robotClimb.moveClimbSeparate(
                    MathUtil.applyDeadband(copilotController.getRightY(), 0.1) * 8,
                    MathUtil.applyDeadband(copilotController.getLeftY(), 0.1) * 8),
                robotClimb));*/
        //SmartDashboard.putNumber("Pigeon Heading", m_robotDrive.getHeading());
        
    }
    private void configureButtonBindings() {
        configureCompetitionButtonBindings();
        //configureTestButtonBindings();
    }

    private void configureTriggers(){
        // Automatically shoot when ready
        new Trigger(() -> 
        DriveConstants.aimedAtTarget && 
        FlywheelConstants.flywheelsAtSetpoint && 
        (FlywheelConstants.currentFlywheelState == FlywheelConstants.flywheelState.SHOOT) && 
        ShooterConstants.armAtSetpoint && 
        (ShooterConstants.currentArmState == ShooterConstants.armState.SPEAKER))
            .onTrue(new commandIndexerStart(robotIndexer));

        // Keep intaking if piece is slipping out
        new Trigger(() ->
            (ShooterConstants.currentArmState == ShooterConstants.armState.SPEAKER ||
             ShooterConstants.currentArmState == ShooterConstants.armState.AMP ||
             ShooterConstants.currentArmState == ShooterConstants.armState.TUNING) &&
            !IndexerConstants.robotHasNote
        ).whileTrue(new commandIndexerPickup(robotIndexer));
    }

    private void configureCompetitionButtonBindings() {
        driverController.x()
            .whileTrue(
                new ParallelCommandGroup(
                    new InstantCommand(() -> m_robotDrive.zeroHeading()),
                    new InstantCommand(() -> poseSubsystem.resetPoseEstimator())
                )
            );
        driverController.y().whileTrue(
            new InstantCommand(() -> m_robotDrive.resetPoseEstimator(defaultPose))
        );
        
        driverController.b().whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> robotShooter.goToTunableSetpoint()),
                new commandFlywheelShoot(robotFlywheels),
                new commandDrivetrainAlignToTarget(m_robotDrive, driverController, poseSubsystem::getPose, poseSubsystem::getTargetYaw)
            )
        );

        // driverController.b().whileTrue(
        //     new InstantCommand(() ->
        //     robotIndexer.getOuttaHere())
        // );
        /*driverController.povRight().whileTrue(
            new InstantCommand(() ->
            robotIndexer.enableBrake())
        );
        driverController.povLeft().whileTrue(
            new InstantCommand(() -> 
            robotIndexer.disableBrake())
        );*/
        //driverController.a().whileTrue(new commandClimbAutoZero(robotClimb));
        /*driverController.b().whileTrue(
            robotLED.redToOrangeTransition()
        );
        driverController.a().whileTrue(
            robotLED.orangeToRedTransition()
        );*/

        // First gets the arm into intake position, then allows the intake and indexer to run
        driverController.rightBumper().whileTrue(
            new SequentialCommandGroup(
                new commandArmIntake(robotShooter),
                new ParallelCommandGroup(
                    new commandIntakePickup(robotIntake),
                    new commandIndexerPickup(robotIndexer)
                ))
            )
            .whileFalse(new commandFlywheelIdle(robotFlywheels));
        
        // First gets the arm into intake position, then allows the intake and indexer to run
        driverController.leftBumper() .whileTrue(
            new SequentialCommandGroup(
                new commandArmIntake(robotShooter),
                new ParallelCommandGroup(
                    new commandIntakeReverse(robotIntake),
                    new commandIndexerReverse(robotIndexer)
                )
            ));

        // When holding the right trigger, the robot will:
        // Have the drivetrain rotate toward the speaker
        // Have the arm get to the correct angle
        // Rev the flywheels up to speed
        /*driverController.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new commandArmAutoAim(robotShooter),
                new commandDrivetrainAimAtSpeaker(m_robotDrive, centralCamera, driverController),
                new commandFlywheelShoot(robotFlywheels),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5), 
                    new commandIndexBrakeMode(robotIndexer)
                )
            ));*/
        driverController.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new commandDrivetrainAlignToTarget(m_robotDrive, driverController, poseSubsystem::getPose, poseSubsystem::getTargetYaw),
                new commandArmAlignToTarget(robotShooter, poseSubsystem::getTargetDistance, centralCamera::getDistanceToTag),
                new commandFlywheelShoot(robotFlywheels)
            )
            
        );


        // Copilot Shooter Commands
        copilotController.x().whileTrue(
            new ParallelCommandGroup(
                robotShooter.goToSetpointCommand(ShooterConstants.kArmParallelPosition),
                new commandFlywheelShoot(robotFlywheels))
            );
        
        copilotController.y().whileTrue(
            new ParallelCommandGroup(
            new commandArmAmp(robotShooter),
            new ParallelDeadlineGroup(
                    new WaitCommand(0.5), 
                    new commandIndexBrakeMode(robotIndexer)
                ))
        );

        copilotController.b().whileTrue(
            new ParallelCommandGroup(
                robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition),
                new commandFlywheelShoot(robotFlywheels),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5), 
                    new commandIndexBrakeMode(robotIndexer)
                )
            ));

        copilotController.a().whileTrue(
            new ParallelCommandGroup(
                robotShooter.goToSetpointCommand(ShooterConstants.kArmShootUnderPosition),
                new commandFlywheelShoot(robotFlywheels)
            ));

        copilotController.leftTrigger().whileTrue(
            new ParallelCommandGroup(
                new commandIndexerStart(robotIndexer),
                new commandIntakeStart(robotIntake)
            ));
        
        copilotController.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new CommandManualFlywheels(robotFlywheels),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new commandIndexerStart(robotIndexer)
                )
            ));

        

        copilotController.povUp()
            .whileTrue(new InstantCommand(() -> robotClimb.setPercentOutput(0.25)))
            .onFalse(new InstantCommand(() -> robotClimb.setPercentOutput(0)));

        copilotController.povDown()
            .whileTrue(new InstantCommand(() -> robotClimb.setPercentOutput(-0.25)))
            .onFalse(new InstantCommand(() -> robotClimb.setPercentOutput(0)));

        copilotController.rightStick()
            .onTrue(new InstantCommand(() -> robotClimb.resetEncoders()));


        // Test trap setpoints.
        copilotController.povRight().whileTrue(robotShooter.goToSetpointCommand(95.0));
        copilotController.povLeft().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmAmpPosition - 8.0));
    }

    private void configureTestButtonBindings() {
        System.out.println("YOU ARE IN TESTING MODE");
        /*driverController.y().whileTrue(new InstantCommand(() -> robotShooter.runManualArmCommand(6)))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));
        driverController.a().whileTrue(new InstantCommand(() -> robotShooter.runManualArmCommand(-12)))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));
        driverController.rightTrigger().whileTrue(new InstantCommand(() -> robotFlywheels.setFlywheelVoltage(12, 6)))
                            .whileFalse(new InstantCommand(() -> robotFlywheels.setFlywheelVoltage(0)));
        driverController.leftTrigger().whileTrue(new commandIndexerPickup(robotIndexer));
        driverController.rightBumper().whileTrue(new commandIntakeStart(robotIntake));
        driverController.b().whileTrue(new InstantCommand(() -> robotShooter.runArmFFOnly()))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));*/
        /*driverController.a().onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> ShooterConstants.armAtSetpoint = true),
            new InstantCommand(() -> FlywheelConstants.flywheelsAtSetpoint = true),
            new InstantCommand(() -> DriveConstants.aimedAtTarget = true)
        ));
        driverController.b().onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> ShooterConstants.armAtSetpoint = false),
            new InstantCommand(() -> FlywheelConstants.flywheelsAtSetpoint = false),
              new InstantCommand(() -> DriveConstants.aimedAtTarget = false)
        ));*/
        driverController.a().whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> ShooterConstants.armAtSetpoint = true),
                new InstantCommand(() -> FlywheelConstants.flywheelsAtSetpoint = true),
                new InstantCommand(() -> DriveConstants.aimedAtTarget = true)
            )
        );
        driverController.b().whileTrue(
            robotLED.colorToRedTransition()
        );
        driverController.x().whileTrue(
            robotLED.colorToOrangeTransition()
        );


    }

    public void registerNamedCommands() {
        //Registers commands for use with pathplanner
        NamedCommands.registerCommand("Subwoofer Shot", 
        new ParallelRaceGroup(
            new WaitCommand(2),
            new ParallelCommandGroup(
                new CommandManualFlywheels(robotFlywheels),
                new SequentialCommandGroup(
                    new commandArmSubwoofer(robotShooter),
                    new WaitCommand(0.2),
                    new commandIndexerStart(robotIndexer)
                ))
        ));
        
        NamedCommands.registerCommand("Prep Subwoofer Shot", new commandFlywheelShoot(robotFlywheels));
        NamedCommands.registerCommand("Index Note", new commandIndexerStart(robotIndexer));
        NamedCommands.registerCommand("Fire Note", new commandIndexerStart(robotIndexer));
        NamedCommands.registerCommand("Pickup Note", 
        new ParallelCommandGroup(
            new commandArmIntake(robotShooter),
            //new ParallelCommandGroup(
                new commandIntakePickup(robotIntake),
                new commandIndexerPickup(robotIndexer)
            //)
        ));
        NamedCommands.registerCommand("Auto Fire", 
        new ParallelRaceGroup(
            new WaitCommand(3), 
            new ParallelCommandGroup(
                new ParallelCommandGroup(
                    new commandArmAutoAim(robotShooter),
                    new commandDrivetrainAimAtSpeaker(m_robotDrive, centralCamera, driverController),
                    new commandFlywheelShoot(robotFlywheels)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(2.0),
                    new commandIndexerStart(robotIndexer)
                )
            )
        ));
        NamedCommands.registerCommand("Prep Fire", 
        new ParallelCommandGroup(
            new commandArmAutoAim(robotShooter),
            new commandFlywheelShoot(robotFlywheels)
        ));
        NamedCommands.registerCommand("EMPTY THE PAYLOAD", 
        new ParallelCommandGroup(
            new commandArmIntake(robotShooter),
            new commandIndexerStart(robotIndexer),
            new commandIntakeStart(robotIntake),
            new CommandManualFlywheels(robotFlywheels)
        ));
    }

    /**
     * 
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getDefaultAutoCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetPoseEstimator(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    public ParallelDeadlineGroup shootNoMoveAuton() {
        //WaitCommand autonWait = new WaitCommand(1);
        //m_robotDrive.
        return 
        new ParallelDeadlineGroup(
            new WaitCommand(5),
            new ParallelCommandGroup(
            new SequentialCommandGroup(
                robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition),
                new WaitCommand(2),
                new commandIndexerStart(robotIndexer)),
            new CommandManualFlywheels(robotFlywheels)
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
