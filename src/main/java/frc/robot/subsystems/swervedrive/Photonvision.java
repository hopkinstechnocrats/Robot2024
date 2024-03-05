package frc.robot.subsystems.swervedrive;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;

public class Photonvision {
    public AprilTagFieldLayout m_aprilTagFieldLayout;
  public PhotonCamera m_cam;
  public PhotonCamera n_cam;
  public PhotonPoseEstimator m_photonPoseEstimator;

  private double m_timestamp = 0.0;

  public Photonvision() {
    try {
      m_aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    // Forward Camera, change name

    m_cam = new PhotonCamera("HD_Pro_Webcam_C920");
    // n_cam = new PhotonCamera("Camera2");
    Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(
                0, 0,
                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.

    // Construct PhotonPoseEstimator
    m_photonPoseEstimator =
        new PhotonPoseEstimator(
            m_aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_cam, robotToCam);

    PortForwarder.add(5800, "HD_Pro_Webcam_C920", 5800);
  }

  public Transform3d GetCamData() {
    Transform3d retval = new Transform3d();

    // Query the latest result from PhotonVision
    var result = m_cam.getLatestResult();

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    // Check to see if the data is new
    boolean isNew = m_timestamp != result.getTimestampSeconds();

    if (hasTargets && isNew) {
      m_timestamp = result.getTimestampSeconds();

      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      double area = target.getArea();
      double pitch = target.getPitch();
      double skew = target.getSkew();
      // Transform2d pose = target.getCameraToTarget();
      // List<TargetCorner> corners = target.getCorners();
      // Get information from target.
      int targetID = target.getFiducialId();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Rotation3d rotator = new Rotation3d(targetID, pitch, yaw);
      // System.out.println(bestCameraToTarget.getZ());
      System.out.println(targetID);
      if (targetID > 0) {
        // Do something!
        // need to retun a "z" value
        retval =
            new Transform3d(
                bestCameraToTarget.getX(),
                bestCameraToTarget.getY(),
                bestCameraToTarget.getZ(),
                rotator);
      } else {
        // System.out.println("no april tag seen");
      }
    }
    return retval;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }  
}
