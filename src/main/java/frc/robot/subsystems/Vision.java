package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class Vision extends SubsystemBase{
    
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;

    private double currentDistance, currentRotation;
    private double currentXDistance, currentYDistance;

    private String cameraName;

    public Vision(String cameraName, Transform3d cameraLocation) {
        this.cameraName = cameraName;
        photonCamera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout,
                                                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                    photonCamera,
                                                    cameraLocation);

        currentDistance = 0;
        currentRotation = 0;
    }

    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      return photonPoseEstimator.update();
    }

    public boolean hasSpeakerTarget() {
        return getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getLatestResult().getBestTarget();
    }
}
