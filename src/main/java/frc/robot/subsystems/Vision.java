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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Vision extends SubsystemBase{
    
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult aprilTagResult;
    private PhotonTrackedTarget trackedTags;
    private Pose3d trackedTagPose;
    private double trackedTagDist;

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

    public boolean hasTargets() {
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

    public Pose3d getGivenTagPose (int i) {
        Pose3d tagPose = aprilTagFieldLayout.getTagPose(i).get();
        return tagPose;
    }

    public Pose3d getEstimatedCameraPose() {
        //var imageCaptureTime = aprilTagResult.getTimestampSeconds();
        Transform3d camToTargetTrans = aprilTagResult.getBestTarget().getBestCameraToTarget();
        Pose3d camPose = trackedTagPose.transformBy(camToTargetTrans.inverse());
        return camPose;
    }

    public double getImageTimestamp() {
        return aprilTagResult.getTimestampSeconds();
    }

    public double getDistanceToTag() {
        return trackedTagDist;
    }

    public Transform3d getCameraLocation() {
        return cameraLocation;
    }

    @Override
    public void periodic() {
        if (this.photonCamera != null) {
            aprilTagResult = getLatestResult();

            if (hasTargets()) {
                for (int i = 0; i < aprilTagResult.getTargets().size(); i++) {
                    int ID = aprilTagResult.getTargets().get(i).getFiducialId();
                    // Come back and set these to be the correct tags, + add the alliance detection.
                    if (ID == 7 || ID == 5 || ID == 3) {
                        trackedTags = aprilTagResult.targets.get(i);
                        trackedTagPose = getGivenTagPose(ID);
                        trackedTagDist = PhotonUtils.calculateDistanceToTargetMeters(
                        cameraLocation.getY(),
                        Units.inchesToMeters(trackedTagPose.getY()), 
                        cameraLocation.getRotation().getY(),
                        Units.degreesToRadians(aprilTagResult.getBestTarget().getYaw()));
                    }
                }
            }
        }
        
        /*for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
            if (i.getFiducialId() != -1) {
                Pose3d aprilTagPose = new Pose3d();
                aprilTagPose = aprilTagFieldLayout.getTagPose(i.getFiducialId()).get();
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            i.getBestCameraToTarget(),
                            aprilTagPose,
                            cameraLocation);
                
            }
        }*/
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
