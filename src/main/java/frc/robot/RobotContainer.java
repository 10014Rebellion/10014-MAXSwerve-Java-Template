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

import frc.robot.subsystems.Shooter.doubleShooterFlywheels;
import frc.robot.subsystems.Shooter.profiledArmPID;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;

//import frc.robot.commands.forceIndexCommand;
import frc.robot.subsystems.Shooter.indexerSubsystem;
import frc.robot.subsystems.Shooter.intakeSubsystem;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.IndexerCommands.commandIndexerReverse;
import frc.robot.commands.IndexerCommands.commandIndexerStart;
import frc.robot.commands.IndexerCommands.commandIndexerPickup;
import frc.robot.commands.IntakeCommands.commandIntakePickup;
import frc.robot.commands.IntakeCommands.commandIntakeStart;
import frc.robot.commands.flywheelCommands.CommandManualFlywheels;
import frc.robot.commands.IntakeCommands.commandIntakeReverse;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.profiledArmPID;
import frc.robot.subsystems.Shooter.shooterFlywheels;
import frc.robot.subsystems.Shooter.doubleShooterFlywheels;
import frc.robot.subsystems.Shooter.indexerSubsystem;
import frc.robot.subsystems.Climb;

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
    private final Climb robotClimb;
    private final doubleShooterFlywheels robotFlywheels;
    private final indexerSubsystem robotIndexer;
    private final intakeSubsystem robotIntake;

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
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    // Copilot's controller
    CommandXboxController copilotController = new CommandXboxController(OIConstants.kCopilotControllerPort);

    // Sets up Auton chooser.
    private final SendableChooser<Command> autoChooser;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Robot subsystem initialization.
        m_robotDrive = new DriveSubsystem();
        robotShooter = new profiledArmPID();
        robotClimb = new Climb();
        robotFlywheels = new doubleShooterFlywheels();
        robotIndexer = new indexerSubsystem();
        robotIntake = new intakeSubsystem();


        //robotIndexer = new indexerCommand();
        //forceRobotIndexer = new forceIndexCommand(robotIndexer);
        //robotIntake = new intakeCommand();

        //commandShoot = new flywheelCommand(robotFlywheels);

        indexAndCheckNoteCommand = new ParallelCommandGroup(new commandIndexerStart(robotIndexer))
                    //.alongWith(new InstantCommand(() -> robotIndexer.ignoreDetection(false)))
                    //.alongWith(robotIndexer)
                    .alongWith(new commandIntakePickup(robotIntake));

        shootSubwooferCommand = new ParallelCommandGroup(robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition), 
                                robotShooter.runFlywheelCommand(12));

        stopAllNoteMotorsCommand = new ParallelCommandGroup(
            robotShooter.runFlywheelCommand(0)
        );
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
                    -MathUtil.applyDeadband(m_driverController.getLeftY()*OIConstants.kDriveMult, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX()*OIConstants.kDriveMult, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX()*OIConstants.kDriveMult*.75, OIConstants.kDriveDeadband),
                    true, false),
                m_robotDrive));

        /*robotArmPID.setDefaultCommand(
            robotArmPID.disablePID()
        );*/
        robotClimb.setDefaultCommand(
            new RunCommand(
                () -> robotClimb.moveClimbSeparate(
                    MathUtil.applyDeadband(copilotController.getLeftY(), 0.1) * 8,
                    MathUtil.applyDeadband(copilotController.getRightY(), 0.1) * 8),
                robotClimb));
        //SmartDashboard.putNumber("Pigeon Heading", m_robotDrive.getHeading());
        
    }
    private void configureButtonBindings() {
        configureCompetitionButtonBindings();
        //configureTestButtonBindings();
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureCompetitionButtonBindings() {
        m_driverController.x()
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));
        
        m_driverController.rightBumper().whileTrue(
            new SequentialCommandGroup(
            new InstantCommand(() -> robotShooter.goToSetpoint(ShooterConstants.kArmIntakePosition)),
            new ParallelCommandGroup(
                new commandIntakePickup(robotIntake),
                new commandIndexerPickup(robotIndexer)
            )));
                                        //.alongWith(robotIntake)))
                                        //.whileFalse(robotIntake.forceRunIntake(0));
                                        //.alongWith(robotIndexer.forceRunIndexer(0)));
        m_driverController.leftBumper() .whileTrue(
            new SequentialCommandGroup(
            new InstantCommand(() -> robotShooter.goToSetpoint(ShooterConstants.kArmIntakePosition)),
            new ParallelCommandGroup(
                new commandIntakeReverse(robotIntake),
                new commandIndexerReverse(robotIndexer)
            )));
                                        //.whileFalse(robotIntake.forceRunIntake(0));
                                        //.alongWith(robotIndexer.forceRunIndexer(0)));

        m_driverController.rightTrigger().whileTrue(new CommandManualFlywheels(robotFlywheels, 12));
        m_driverController.leftTrigger().whileTrue(
                new ParallelCommandGroup(
                    new commandIndexerStart(robotIndexer),
                    new commandIntakeStart(robotIntake)
                    )
            );
        // Copilot Shooter Commands
        copilotController.x().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition));
        
        copilotController.y().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmAmpPosition));

        copilotController.b().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferSideShotPosition));

        copilotController.a().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmYeetPosition));

        /*copilotController.rightTrigger().whileTrue(robotShooter.runFlywheelCommand(12)
                                                .alongWith(robotIndexer.forceRunIndexer(6)))
                                    .whileFalse(robotIndexer.forceRunIndexer(0)
                                                .alongWith(robotShooter.runFlywheelCommand(0)));

        copilotController.leftTrigger().whileTrue(robotShooter.runFlywheelCommand(12)
                                                .alongWith(robotIndexer.forceRunIndexer(-6)))
                                    .whileFalse(robotIndexer.forceRunIndexer(0)
                                                .alongWith(robotShooter.runFlywheelCommand(0)));
*/
        copilotController.povUp().whileTrue(robotShooter.goToSetpointCommand(ShooterConstants.kArmDefensePosition));
        //copilotController.povDown().whileTrue());
        
        }

    private void configureTestButtonBindings() {
        System.out.println("YOU ARE IN TESTING MODE");
        m_driverController.y().whileTrue(new InstantCommand(() -> robotShooter.runManualArmCommand(6)))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));
        m_driverController.a().whileTrue(new InstantCommand(() -> robotShooter.runManualArmCommand(-12)))
                                .whileFalse(new InstantCommand(() -> robotShooter.runManualArmCommand(0)));
        m_driverController.rightTrigger().whileTrue(new InstantCommand(() -> robotFlywheels.setFlywheelVoltage(12, 6)))
                            .whileFalse(new InstantCommand(() -> robotFlywheels.setFlywheelVoltage(0)));
        m_driverController.leftTrigger().whileTrue(new commandIndexerPickup(robotIndexer));
        m_driverController.rightBumper().whileTrue(new commandIntakeStart(robotIntake));
        m_driverController.b().whileTrue(new InstantCommand(() -> robotShooter.runArmFFOnly()))
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
        return robotShooter.goToSetpointCommand(ShooterConstants.kArmSubwooferShotPosition - 5)
                .andThen(new WaitCommand(2))
                .andThen(robotShooter.runFlywheelCommand(12)
                .alongWith(new WaitCommand(1)))
                //.andThen(robotIndexer.forceRunIndexer(6))
                .andThen(new WaitCommand(1))
                .andThen(robotShooter.runFlywheelCommand(0));
                //.alongWith(robotIndexer.forceRunIndexer(0)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
