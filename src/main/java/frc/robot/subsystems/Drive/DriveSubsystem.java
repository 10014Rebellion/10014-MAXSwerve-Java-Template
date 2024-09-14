// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.photonConstants;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);



  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonCanID, "rio");

  // Turning PID Controller
  

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private Field2d field = new Field2d();
  private Vision centralCamera;

  Vector<N3> stateStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01)); // Increase for less trust
  Vector<N3> visionStdDevs = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(0.2)); // Increase for less vision trust

  private SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics, 
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      }, DriverStation.getAlliance().toString().equals("Red")
      ? DriveConstants.kInitialRedPose
      : DriveConstants.kInitialBluePose,
      stateStdDevs,
      visionStdDevs);

  private PIDController turnController;

  public DriveSubsystem(Vision centralCamera) {
    this.centralCamera = centralCamera;
    m_gyro.setYaw(swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    AutoBuilder.configureHolonomic(this::getPose, this::resetPoseEstimator,
     this::getChassisSpeed, this::driveRobotRelative, 
     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4759, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
    ),  () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE S
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        //System.out.println("CURRENT ALLIANCE: " + alliance.get());
        return alliance.get() == DriverStation.Alliance.Red;
      }
      else return false;
    },
    this // Reference to this subsystem to set requirements);
    );
    turnController = new PIDController(DriveConstants.kAngularP, 0, DriveConstants.kAngularD);
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    swervePoseEstimator.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
       }
    );
    
    var visionResult = centralCamera.getLatestResult();
    Optional<EstimatedRobotPose> centralCamPoseEstimate = centralCamera.getEstimatedGlobalPose(swervePoseEstimator.getEstimatedPosition());
    if(centralCamPoseEstimate.isPresent())  {
      swervePoseEstimator.addVisionMeasurement(centralCamPoseEstimate.get().estimatedPose.toPose2d(), visionResult.getTimestampSeconds());
    }

    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Estimated Tag Distance", centralCamera.getDistanceToTag());
    SmartDashboard.putNumber("Drive Back Left Motor Temp", m_rearLeft.getMotorTemp());
    SmartDashboard.putNumber("Drivetrain Yaw", swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    //SmartDashboard.putNumber("Drivetrain Gyro Yaw", m_gyro.getRotation2d().getDegrees());
    //photonConstants.speakerDistance = getDistanceToPose(FieldConstants.kBlueSpeakerAprilTagLocation);
    SmartDashboard.putNumber("Rotation To Hit Blue speaker", getRotationToPose(FieldConstants.kBlueSpeakerAprilTagLocation));
    SmartDashboard.putNumber("Distance To Blue Speaker", getDistanceToPose(FieldConstants.kBlueSpeakerAprilTagLocation));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPoseEstimator(Pose2d pose) {
        swervePoseEstimator.resetPosition(
        m_gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }, 
        pose);
  }

  public double getRotationToPose(Pose2d target) {
    double robotPoseX = swervePoseEstimator.getEstimatedPosition().getX();
    double robotPoseY = swervePoseEstimator.getEstimatedPosition().getY();
    double targetPoseX = target.getX();
    double targetPoseY = target.getY();
    double rotationToPose = Math.atan2(targetPoseY-robotPoseY, targetPoseX-robotPoseX);
    return rotationToPose;
  }
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
  }

  public double getDistanceToPose(Pose2d target) {
    double robotPoseX = swervePoseEstimator.getEstimatedPosition().getX();
    double robotPoseY = swervePoseEstimator.getEstimatedPosition().getY();
    double targetPoseX = target.getX();
    double targetPoseY = target.getY();
    double targetDistance = Math.sqrt(Math.pow(robotPoseX-targetPoseX,2) + Math.pow(robotPoseY - targetPoseY,2));
    return targetDistance;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;
    double rateLimitEnablerVal = OIConstants.kDriveDeadband * OIConstants.kDriveMult;
    if (rateLimit && ((Math.abs(xSpeed) > rateLimitEnablerVal) || (Math.abs(ySpeed) > rateLimitEnablerVal) || (Math.abs(rot) > rateLimitEnablerVal))) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered;
    // Tells the drivetrain to use pid instead of normal control if it's trying to aim at something.
    if (DriveConstants.currentDriveState == DriveConstants.driveState.AIMING) {
      double goalYaw = centralCamera.getYaw();
      m_currentRotation = turnController.calculate(centralCamera.getYaw(), 0);
      DriveConstants.aimedAtTarget = (Math.abs(goalYaw) < 2);
      SmartDashboard.putNumber("Calculated Turn Speed: camera", m_currentRotation);
      SmartDashboard.putNumber("Calculated Turn Speed: Pose Estimator", turnController.calculate(m_gyro.getAngle(), getRotationToPose(FieldConstants.kBlueSpeakerAprilTagLocation)));
    }
    rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    //SmartDashboard.putRaw("Front left state", m_frontLeft.getState());
    SmartDashboard.putNumber("Pigeon Heading", m_gyro.getAngle());
    SmartDashboard.putNumber("RearLeft motor temp", m_rearLeft.getMotorTemp());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    speeds = ChassisSpeeds.discretize(speeds, .02);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
}

  private ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
    };
}

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    
    m_gyro.reset();
    swervePoseEstimator.resetPosition(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
      new Pose2d(swervePoseEstimator.getEstimatedPosition().getTranslation(), m_gyro.getRotation2d())
    );
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public void setHeading(Pose2d pose) {
    m_gyro.setYaw(pose.getRotation().getDegrees());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /*public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }*/
}
