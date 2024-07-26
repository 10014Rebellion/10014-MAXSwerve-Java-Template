package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Vision extends SubsystemBase{
    
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;

    private Transform3d cameraLocation;
    private AprilTagFieldLayout aprilTagFieldLayout;

    private double currentDistance, currentRotation;
    private double currentXDistance, currentYDistance;

    private String cameraName;

    //private Field2d field = new Field2d();

    public Vision(String cameraName, Transform3d cameraLocation) {
        this.cameraName = cameraName;
        photonCamera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout,
                                                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                    photonCamera,
                                                    cameraLocation);
        this.cameraLocation = cameraLocation;
        aprilTagFieldLayout = FieldConstants.kAprilTagFieldLayout;
        currentDistance = 0;
        currentRotation = 0;
        
        //SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("pv X", 0);
        SmartDashboard.putNumber("pv Y", 0);
    }

    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevPoseEstimate) {
        photonPoseEstimator.setReferencePose(prevPoseEstimate);
        return photonPoseEstimator.update();
    }

    public Pose3d apriltagRelativeFieldPose () {
        Pose3d robotPose = new Pose3d();
        for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
            if (i.getFiducialId() != -1) {
                Pose3d aprilTagPose = new Pose3d();
                aprilTagPose = aprilTagFieldLayout.getTagPose(i.getFiducialId()).get();
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            i.getBestCameraToTarget(),
                            aprilTagPose,
                            cameraLocation);
                
            }
        }
        return robotPose;
    }

    public boolean hasSpeakerTarget() {
        return getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getLatestResult().getBestTarget();
    }

    public int getCurrentID() {
        var result = getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getFiducialId();
        }
        else return -1;
    }

    @Override
    public void periodic() {
        /*Optional<EstimatedRobotPose> photonPoseEstimation = getEstimatedGlobalPose();
        photonPoseEstimation.ifPresent(poseEstimation -> {
            SmartDashboard.putNumber("pv X", poseEstimation.estimatedPose.getX());
            SmartDashboard.putNumber("pv Y", poseEstimation.estimatedPose.getY());
        });
        //var result
        /*if (photonPoseEstimation.isPresent()) {
            
        }
        var result = photonCamera.getLatestResult();
        SmartDashboard.putBoolean("I SEE YOU", result.hasTargets());
        if (result.hasTargets()) {
            SmartDashboard.putNumber("I see this Apriltag number", result.getBestTarget().getFiducialId());
        }
        
        //Pose3d visionMeasurement3d = 
        //objectToRobo*/

        

        
    }
}
