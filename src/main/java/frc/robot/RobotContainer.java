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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.photonConstants;
import frc.robot.subsystems.Shooter.profiledArmPID;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LEDInterface;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.pidClimbSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.indexerSubsystem;
import frc.robot.subsystems.Shooter.intakeSubsystem;

//import frc.robot.commands.forceIndexCommand;
import frc.robot.commands.IndexerCommands.commandIndexerReverse;
import frc.robot.commands.IndexerCommands.commandIndexerStart;
import frc.robot.commands.ArmCommands.commandArmAmp;
import frc.robot.commands.ArmCommands.commandArmAutoAim;
import frc.robot.commands.ArmCommands.commandArmIntake;
import frc.robot.commands.ClimbCommands.commandClimbAutoZero;
import frc.robot.commands.DriveCommands.commandDrivetrainAimAtSpeaker;
import frc.robot.commands.IndexerCommands.commandIndexerPickup;
import frc.robot.commands.IntakeCommands.commandIntakePickup;
import frc.robot.commands.IntakeCommands.commandIntakeStart;
import frc.robot.commands.LEDCommands.commandLEDNotePickup;
import frc.robot.commands.flywheelCommands.commandManualFlywheels;
import frc.robot.commands.flywheelCommands.commandFlywheelIdle;
import frc.robot.commands.flywheelCommands.commandFlywheelShoot;
import frc.robot.commands.IntakeCommands.commandIntakeReverse;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    private final pidClimbSubsystem robotClimb;
    private final doubleShooterFlywheels robotFlywheels;
    private final indexerSubsystem robotIndexer;
    private final intakeSubsystem robotIntake;

    private final Vision centralCamera;
    private final LEDInterface robotLED;

    //private final indexerCommand robotIndexer = new indexerCommand();
    //private final forceIndexCommand forceRobotIndexer = new forceIndexCommand(robotIndexer);
    //private final intakeCommand robotIntake = new intakeCommand();

    //private final indexerCommand robotIndexer;
    //private final forceIndexCommand forceRobotIndexer;
    //private final flywheelCommand commandShoot;

    private ParallelCommandGroup indexAndCheckNoteCommand;
    private ParallelCommandGroup shootSubwooferCommand;
    private ParallelCommandGroup stopAllNoteMotorsCommand;

                                                        

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
        robotClimb = new pidClimbSubsystem();
        robotFlywheels = new doubleShooterFlywheels();
        robotIndexer = new indexerSubsystem();
        robotIntake = new intakeSubsystem();
        robotLED = new LEDInterface();
        
        


        //robotIndexer = new indexerCommand();
        //forceRobotIndexer = new forceIndexCommand(robotIndexer);
        //robotIntake = new intakeCommand();

        //commandShoot = new flywheelCommand(robotFlywheels);

        indexAndCheckNoteCommand = new ParallelCommandGroup(new commandIndexerStart(robotIndexer))
                    //.alongWith(new InstantCommand(() -> robotIndexer.ignoreDetection(false)))
                    //.alongWith(robotIndexer)
                    .alongWith(new commandIntakePickup(robotIntake));

        shootSubwooferCommand = new ParallelCommandGroup(robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition), 
                                new commandManualFlywheels(robotFlywheels));
        //ParallelCommandGroup prepSubwooferCommand = new ParallelCommandGroup( );
        //Registers commands for use with pathplanner
        NamedCommands.registerCommand("Prep Subwoofer Shot", shootSubwooferCommand);
        NamedCommands.registerCommand("Index Note", new commandIndexerStart(robotIndexer));
        //NamedCommands.registerCommand("Stop Pickup", new InstantCommand(robotIntake.end(true)_);
        NamedCommands.registerCommand("Pickup Note", indexAndCheckNoteCommand);
        NamedCommands.registerCommand("Go To Subwoofer Shot Position", robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition));
        // Configure the button bindings
        configureButtonBindings();

        // Auton Sendable Chooser
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

    private void configureCompetitionButtonBindings() {
        driverController.x()
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));
        driverController.y().whileTrue(
            new InstantCommand(() -> robotLED.resetStartingLED())
        );
        driverController.b().whileTrue(
            new InstantCommand(() -> robotShooter.goToTunableSetpoint())
        );
        driverController.a().whileTrue(
            robotShooter.goToSetpointCommand(0)
        );
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
                    new commandIndexerPickup(robotIndexer),
                    new commandLEDNotePickup(robotLED)
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
        driverController.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new commandArmAutoAim(robotShooter),
                new commandDrivetrainAimAtSpeaker(m_robotDrive, centralCamera, driverController),
                new commandFlywheelShoot(robotFlywheels)
            ));


        // Copilot Shooter Commands
        copilotController.x().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition));
        
        copilotController.y().whileTrue(new commandArmAmp(robotShooter));

        copilotController.b().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmParallelPosition));

        copilotController.a().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmYeetPosition));

        copilotController.leftTrigger().whileTrue(
            new ParallelCommandGroup(
                new commandIndexerStart(robotIndexer),
                new commandIntakeStart(robotIntake)
            ));
        
        copilotController.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new commandManualFlywheels(robotFlywheels),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new commandIndexerStart(robotIndexer)
                )
            ));

        copilotController.leftBumper().whileTrue(
            new InstantCommand(() -> robotClimb.moveLeftClimb(0.1)))
        .whileFalse(new InstantCommand(() -> robotClimb.moveLeftClimb(0)));
        
        copilotController.rightBumper().whileTrue(
             new InstantCommand(() -> robotClimb.moveRightClimb(0.1)))
        .whileFalse(new InstantCommand(() -> robotClimb.moveRightClimb(0)));

        copilotController.povUp().whileTrue(new InstantCommand(() -> robotClimb.moveBothClimb(0.25)))
                                .whileFalse(new InstantCommand(() -> robotClimb.moveBothClimb(0)));

        copilotController.povDown().whileTrue(new InstantCommand(() -> robotClimb.moveBothClimb(-0.5)))
                                .whileFalse(new InstantCommand(() -> robotClimb.moveBothClimb(0)));
        // Test trap setpoints.
        copilotController.povRight().whileTrue(robotShooter.goToSetpointCommand(80.0));
        copilotController.povLeft().whileTrue(robotShooter.goToSetpointCommand(70.0));
    }

    private void configureTestButtonBindings() {
        System.out.println("YOU ARE IN TESTING MODE");
        driverController.y().whileTrue(new InstantCommand(() -> robotShooter.runManualArmCommand(6)))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));
        driverController.a().whileTrue(new InstantCommand(() -> robotShooter.runManualArmCommand(-12)))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));
        driverController.rightTrigger().whileTrue(new InstantCommand(() -> robotFlywheels.setFlywheelVoltage(12, 6)))
                            .whileFalse(new InstantCommand(() -> robotFlywheels.setFlywheelVoltage(0)));
        driverController.leftTrigger().whileTrue(new commandIndexerPickup(robotIndexer));
        driverController.rightBumper().whileTrue(new commandIntakeStart(robotIntake));
        driverController.b().whileTrue(new InstantCommand(() -> robotShooter.runArmFFOnly()))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));
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
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    public SequentialCommandGroup shootNoMoveAuton() {
        //WaitCommand autonWait = new WaitCommand(1);
        //m_robotDrive.
        return new SequentialCommandGroup(
            robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition - 5),
            new WaitCommand(2),
            new commandManualFlywheels(robotFlywheels),
            new WaitCommand(1),
            new commandIndexerStart(robotIndexer));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
