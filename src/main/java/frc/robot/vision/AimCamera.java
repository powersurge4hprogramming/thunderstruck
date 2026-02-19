package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class AimCamera {
    // =================================================================================================================
    // Constants
    // =================================================================================================================
    private final static byte HUB_CENTER_TAG = 9;
    private final static byte HUB_OFF_CENTER_RIGHT_TAG = 10;
    private final static Transform3d SHOOTER_TO_CAMERA_OFFSET = new Transform3d(
            // x
            Distance.ofBaseUnits(5, Inches),
            // y
            Distance.ofBaseUnits(0, Inches),
            // z
            Distance.ofBaseUnits(-12, Inches),
            new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees)));
    private static final Transform3d ROBOT_TO_CAMERA_OFFSET = new Transform3d(
            // x
            Distance.ofBaseUnits(5, Inches),
            // y
            Distance.ofBaseUnits(0, Inches),
            // z
            Distance.ofBaseUnits(-12, Inches),
            new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees)));

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);
    public static final Vector<N3> visionStdDevs = VecBuilder.fill(0.1, 0.1, 0.01);

    // =================================================================================================================
    // Private Members
    // =================================================================================================================
    private final PhotonPoseEstimator photonPoseEstimator;
    private List<PhotonPipelineResult> results;

    // =================================================================================================================
    // Systems
    // =================================================================================================================
    private final PhotonCamera camera;

    // =================================================================================================================
    // Public Parts
    // =================================================================================================================
    public AimCamera() {
        this.camera = new PhotonCamera("photonvision");
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_CAMERA_OFFSET);
    }

    // -----------------------------------------------------------------------------------------------------------------
    public void updateFrames() {
        this.results = camera.getAllUnreadResults();
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * This gets the {@link Transform3d} when the camera is pointed at the Hub
     * AprilTag(s).
     * 
     * @apiNote
     *          This should be called when the camera is already
     *          pointing at the Hub.
     * 
     * @return The {@link Transform3d} when the camera is pointed at the Hub, or
     *         null if it isn't.
     */
    public Transform3d getHubRelativeLocation() {
        Transform3d hub = null;

        for (final PhotonPipelineResult result : results) {
            for (final PhotonTrackedTarget target : result.getTargets()) {
                if (target.fiducialId != HUB_CENTER_TAG || target.fiducialId != HUB_OFF_CENTER_RIGHT_TAG)
                    continue;
                hub = target.getBestCameraToTarget();
                break;
            }
        }

        if (hub != null)
            hub.plus(SHOOTER_TO_CAMERA_OFFSET);

        return hub;
    }

    // -----------------------------------------------------------------------------------------------------------------
    public void updateEstimatedRobotPose(final Consumer<VisionMeasurement> poseUpdator) {
        for (final PhotonPipelineResult result : results) {
            if (!result.hasTargets())
                continue;
            Optional<EstimatedRobotPose> optionalPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            if (optionalPose.isEmpty())
                continue;

            EstimatedRobotPose pose = optionalPose.get();
            /*
             * Dynamic std devs: Scale trust based on tag count/distance
             * (lower = more trust).
             */
            Vector<N3> dynamicStdDevs = visionStdDevs;
            if (result.getTargets().size() == 1) {
                dynamicStdDevs = VecBuilder.fill(0.2, 0.2, 0.02); // Less trust with single tag
            } else if (pose.estimatedPose.getTranslation().getNorm() > 5.0) { // Example: Far away
                dynamicStdDevs = VecBuilder.fill(0.15, 0.15, 0.015);
            }

            final VisionMeasurement measurement = new VisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    result.getTimestampSeconds(),
                    dynamicStdDevs);
            if (result.getTimestampSeconds() < 0.5) {
                poseUpdator.accept(measurement);
            }
        }
    }

}
