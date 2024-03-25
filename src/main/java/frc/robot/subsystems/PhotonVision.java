package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class PhotonVision extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("photonvision");

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    Transform3d cameraToRobot = new Transform3d();

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            double photonYaw = target.getYaw();
            SmartDashboard.putNumber("photonYaw", photonYaw);

            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                    cameraToRobot);
            SmartDashboard.putNumber("Robot Pose X", robotPose.getX());
            SmartDashboard.putNumber("Robot Pose Y", robotPose.getY());
            SmartDashboard.putNumber("Robot Pose Z", robotPose.getZ());
            SmartDashboard.putNumber("Robot Angle Roll", robotPose.getRotation().getX() * (180 / Math.PI));
            SmartDashboard.putNumber("Robot Angle Pitch", robotPose.getRotation().getY() * (180 / Math.PI));
            SmartDashboard.putNumber("Robot Angle Yaw", robotPose.getRotation().getZ() * (180 / Math.PI));

            int targetID = target.getFiducialId();
            SmartDashboard.putNumber("targetID", targetID);   

            double poseAmbiguity = target.getPoseAmbiguity();
            SmartDashboard.putNumber("poseAmbiguity", poseAmbiguity);

            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            SmartDashboard.putNumber("bestCameraToTargetX", bestCameraToTarget.getX());
            SmartDashboard.putNumber("bestCameraToTargetY", bestCameraToTarget.getY());
            SmartDashboard.putNumber("bestCameraToTargetZ", bestCameraToTarget.getZ());
        }
    }
}
