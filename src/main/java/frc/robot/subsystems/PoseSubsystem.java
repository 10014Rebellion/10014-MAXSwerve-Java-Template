package frc.robot.subsystems;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.photonConstants;
import frc.robot.utils.Utils;

public class PoseSubsystem extends SubsystemBase{
    private final Vision centralCamera;
    private final Supplier<Rotation2d> gyroRotation;
    private final Supplier<SwerveModulePosition[]> swerveModulePositions;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    public PoseSubsystem(Vision centralCamera, Supplier<Rotation2d> gyroRotation, Supplier<SwerveModulePosition[]> swerveModulePositions) {
        this.centralCamera = centralCamera;
        this.gyroRotation = gyroRotation;
        this.swerveModulePositions = swerveModulePositions;
        poseEstimator = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics,
        gyroRotation.get(),
        swerveModulePositions.get(),
        new Pose2d()
        );
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
      updatePose();
      updateTelemetry();
      updateDistanceToSpeaker();
    }

    public void updatePose() {
        poseEstimator.update(gyroRotation.get(), swerveModulePositions.get());
        centralCamera.getEstimatedGlobalPose().ifPresent(estimatedRobotPose -> {
            Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
            if (isPoseOnField(pose)) {
                if (estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                  poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, Constants.photonConstants.kVisionMultiTagStandardDeviations);
                  }
                else {
                    for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                        if (Utils.isValueInRange(target.getPoseAmbiguity(), 0.0, Constants.photonConstants.kVisionMaxPoseAmbiguity)) {
                          poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, Constants.photonConstants.kVisionSingleTagStandardDeviations);
                          break;
                        }
                      }
                }
            }
        });
    }

    public void forceResetPose() {
      poseEstimator.update(gyroRotation.get(), swerveModulePositions.get());
        centralCamera.getEstimatedGlobalPose().ifPresent(estimatedRobotPose -> {
            Pose2d camPose = estimatedRobotPose.estimatedPose.toPose2d();
            poseEstimator.resetPosition(gyroRotation.get(), swerveModulePositions.get(), camPose);
            //if (isPoseOnField(camPose)) {
                //if (estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                  //poseEstimator.resetPosition(gyroRotation.get(), swerveModulePositions.get(), pose);
                  //}            
           // }
    });
  }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose3d getTargetPose() {
        return Utils.getValueForAlliance(
          Constants.FieldConstants.kBlueSpeaker,
          Constants.FieldConstants.kRedSpeaker);
      }
    
      public double getTargetYaw() {
        Pose2d robotPose = getPose();
        Pose2d targetPose = getTargetPose().toPose2d().transformBy(
          new Transform2d(
            Utils.getValueForAlliance(
              Constants.FieldConstants.kSpeakerTargetYawTransformX, 
              -Constants.FieldConstants.kSpeakerTargetYawTransformX
            ), 
            Constants.FieldConstants.kSpeakerTargetYTransform, 
            Rotation2d.fromDegrees(0.0)
          )
        );
        Translation2d targetTranslation = targetPose.relativeTo(robotPose).getTranslation();
        Rotation2d targetRotation = new Rotation2d(targetTranslation.getX(), targetTranslation.getY());
        targetRotation = targetRotation
          .rotateBy(Rotation2d.fromDegrees(180.0))
          .rotateBy(robotPose.getRotation());
        return Utils.wrapAngle(targetRotation.getDegrees());
      }
    
      public double getTargetPitch() {
        return Utils.getPitchToPose(
          new Pose3d(getPose()),
          getTargetPose().transformBy(
            new Transform3d(
              0.0, 0.0, 
              Constants.FieldConstants.kSpeakerTargetPitchTransformZ,
              new Rotation3d()
            )
          )
        );
      }
      public double getTargetDistance() {
        return Utils.getDistanceToPose(
          getPose(),
          getTargetPose().toPose2d().transformBy(
            new Transform2d(
              Utils.getValueForAlliance(
                -Constants.FieldConstants.kSpeakerTargetDistanceTransformX, 
                Constants.FieldConstants.kSpeakerTargetDistanceTransformX
              ), 
              0.0, Rotation2d.fromDegrees(0.0)
            )
          )
        );
      }

    public void updateDistanceToSpeaker() {
        photonConstants.speakerDistance = getTargetDistance();
      }

    public void updateTelemetry() {
        Pose2d robotPose = getPose();
        field.setRobotPose(robotPose);
        SmartDashboard.putNumberArray("Robot/Pose/Current", new double[] { robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees() });
        SmartDashboard.putNumber("Robot/Pose/Target/Yaw", getTargetYaw());
        SmartDashboard.putNumber("Robot/Pose/Target/Pitch", getTargetPitch());
        SmartDashboard.putNumber("Robot/Pose/Target/Distance", getTargetDistance());
    }
    
    public void resetPoseEstimator() {
      poseEstimator.resetPosition(
        gyroRotation.get(), 
        swerveModulePositions.get(),
        new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), gyroRotation.get())
    );
    }
    public void resetPoseEstimator(Pose2d robotPose) {
      poseEstimator.resetPosition(
        gyroRotation.get(), 
        swerveModulePositions.get(),
        robotPose);
    }


    private boolean isPoseOnField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return (x >= 0.0 && x <= Constants.FieldConstants.kAprilTagFieldLayout.getFieldLength()) && (y >= 0.0 && y <= Constants.FieldConstants.kAprilTagFieldLayout.getFieldWidth());
    }
}
