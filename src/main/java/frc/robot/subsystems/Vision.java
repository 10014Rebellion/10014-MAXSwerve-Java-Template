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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.photonConstants;

public class Vision extends SubsystemBase{
    
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult aprilTagResult;
    private PhotonTrackedTarget trackedTag;
    private Pose3d trackedTagPose;
    private double trackedTagDist;

    private Transform3d cameraLocation;
    private AprilTagFieldLayout aprilTagFieldLayout;
    

    private double currentRotation;
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
        trackedTagDist = 0;
        currentRotation = 0;
        
        //SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("pv X", 0);
        SmartDashboard.putNumber("pv Y", 0);
    }

    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousPose) {
        photonPoseEstimator.setReferencePose(previousPose);
        return photonPoseEstimator.update();
    }

    public double getImageTimestamp() {
        return aprilTagResult.getTimestampSeconds();
    }

    public void updateDistanceToTag() {
        trackedTagDist = PhotonUtils.calculateDistanceToTargetMeters(
            cameraLocation.getZ(), 
            trackedTagPose.getZ(),
            cameraLocation.getRotation().getY(),
            Units.degreesToRadians(aprilTagResult.getBestTarget().getPitch()));
        photonConstants.speakerDistance = trackedTagDist;
    }

    public double getDistanceToTag() {
        return trackedTagDist;
    }

    public Transform3d getCameraLocation() {
        return cameraLocation;
    }

    public double getYaw() {
        if (aprilTagResult.hasTargets()) {
            for (PhotonTrackedTarget i : aprilTagResult.getTargets()) {
                if (i.getFiducialId() == 4) { //&& DriverStation.getAlliance().toString().equals("Red"))
                    return i.getYaw();
                }
                else if (i.getFiducialId() == 7 ) {//&& DriverStation.getAlliance().toString().equals("Blue")) 
                    return i.getYaw();
                }
            }
        }
        return 0.0;
    }

    @Override
    public void periodic() {
        aprilTagResult = getLatestResult();
        if(aprilTagResult.hasTargets()) {
            for (PhotonTrackedTarget i : aprilTagResult.getTargets()) {
                // These IDs are only the central speakers for each alliance.
                if (i.getFiducialId() == 3) { //&& DriverStation.getAlliance().toString().equals("Red")) {
                    trackedTag = i;
                    trackedTagPose = aprilTagFieldLayout.getTagPose(3).get();
                    updateDistanceToTag();
                }
                else if (i.getFiducialId() == 7 ) {//&& DriverStation.getAlliance().toString().equals("Blue")) {
                    trackedTag = i;
                    trackedTagPose = aprilTagFieldLayout.getTagPose(7).get();
                    updateDistanceToTag();
                }
            }
        }
        SmartDashboard.putNumber("Robot Distance from Speaker (Meters)", trackedTagDist);
    }
}
