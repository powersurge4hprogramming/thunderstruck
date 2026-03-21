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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimCamera {
    // =================================================================================================================
    // Constants
    // =================================================================================================================
    private final static byte HUB_OFF_CENTER_RIGHT_RED_TAG = 9;
    private final static byte HUB_CENTER_RED_TAG = 10;
    private final static byte HUB_CENTER_BLUE_TAG = 26;
    private final static byte HUB_OFF_CENTER_LEFT_BLUE_TAG = 25;

    /**
     * Transform from the shooter origin to the camera origin (robot frame).
     * <p>
     * x = −27.5 in → camera is 27.5 inches behind the shooter<br>
     * y = −2 in → camera is 2 inches to the right<br>
     * z = +3.7 in → camera is 3.7 inches above
     * <p>
     * Zero rotation → camera and robot frames are parallel.
     * This is critical: it means SHOOTER_TO_CAMERA.plus(cameraToTag) does
     * NOT rotate the camera-to-tag translation, only adds the offset.
     */
    private final static Transform3d SHOOTER_TO_CAMERA_OFFSET = new Transform3d(
            Distance.ofRelativeUnits(27.5, Inches),
            Distance.ofRelativeUnits(-2, Inches),
            Distance.ofRelativeUnits(3.70, Inches),
            new Rotation3d(
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees)));

    private static final Transform3d ROBOT_TO_CAMERA_OFFSET = new Transform3d(
            Distance.ofRelativeUnits(25, Inches),
            Distance.ofRelativeUnits(-2, Inches),
            Distance.ofRelativeUnits(23.5, Inches),
            new Rotation3d(
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees)));

    /**
     * Offset from Tag 9 (off-center left) to the hub scoring center,
     * expressed in the <b>tag's coordinate frame</b>.
     * <p>
     * After fixing the Transform3d composition order, this offset is
     * rotated from the tag frame into the robot frame automatically
     * by the camera-to-tag rotation. This means the same offset works
     * correctly regardless of the robot's viewing angle.
     * <p>
     * <b>⚠️ These values MUST be re-measured after the composition fix.
     * The old values were tuned with broken code that flipped signs.</b>
     */
    private static final Transform3d TAG9_TO_HUB_CENTER_OFFSET = new Transform3d(
            // Tag frame (WPILib convention):
            // X = out of tag face (toward camera)
            // Y = left when viewing tag from front
            // Z = up
            // Positive X = toward camera, negative X = into hub.
            Distance.ofRelativeUnits(-23.5, Inches),
            Distance.ofRelativeUnits(15, Inches),
            Distance.ofRelativeUnits(27.5, Inches),
            new Rotation3d(
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees)));

    /**
     * Offset from Tag 10 (center) to the hub scoring center,
     * expressed in the <b>tag's coordinate frame</b>.
     */
    private static final Transform3d TAG10_TO_HUB_CENTER_OFFSET = new Transform3d(
            Distance.ofRelativeUnits(-23.5, Inches),
            Distance.ofRelativeUnits(0, Inches),
            Distance.ofRelativeUnits(27.5, Inches),
            new Rotation3d(
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees)));

    private static final Transform3d TAG25_TO_HUB_CENTER_OFFSET = new Transform3d(
            Distance.ofRelativeUnits(-23.5, Inches),
            Distance.ofRelativeUnits(15, Inches),
            Distance.ofRelativeUnits(27.5, Inches),
            new Rotation3d(
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees)));

    private static final Transform3d TAG26_TO_HUB_CENTER_OFFSET = new Transform3d(
            Distance.ofRelativeUnits(23.5, Inches),
            Distance.ofRelativeUnits(0, Inches),
            Distance.ofRelativeUnits(27.5, Inches),
            new Rotation3d(
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees)));

    private SendableChooser<AprilTagFields> fieldChooser = new SendableChooser<>();
    private AprilTagFields lastField = null;

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
        fieldChooser.setDefaultOption("Welded", AprilTagFields.k2026RebuiltWelded);
        fieldChooser.addOption("AndyMark", AprilTagFields.k2026RebuiltAndymark);
        SmartDashboard.putData("Field Layout", fieldChooser);
        this.camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        AprilTagFields initial = fieldChooser.getSelected();
        if (initial == null) {
            initial = AprilTagFields.k2026RebuiltWelded;
        }
        lastField = initial;
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(initial),
                ROBOT_TO_CAMERA_OFFSET);
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * Must be called <b>every</b> frame in the robot's periodic function.
     */
    public void updateFrames() {
        this.results = camera.getAllUnreadResults();

        AprilTagFields selected = fieldChooser.getSelected();
        if (selected != null && selected != lastField) {
            lastField = selected;
            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(selected);
            photonPoseEstimator.setFieldTags(layout);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * Returns the Transform3d from the <b>shooter</b> to the hub scoring
     * center, in the <b>robot's coordinate frame</b>. Returns null if no
     * hub tag is visible.
     *
     * <h3>Frame chain (the fix):</h3>
     * 
     * <pre>
     *   Shooter ──→ Camera ──→ Tag ──→ Hub Center
     *     (known)    (vision)   (known)
     *
     *   result = SHOOTER_TO_CAMERA
     *              .plus(cameraToTag)
     *              .plus(tagToHubOffset)
     * </pre>
     *
     * <h3>Why the old code was wrong:</h3>
     * 
     * <pre>
     *   OLD: cameraToTag.plus(SHOOTER_TO_CAMERA).plus(tagToHub)
     *
     *   .plus() rotates the second operand's translation by the
     *   first operand's rotation. cameraToTag has a large rotation
     *   (the tag faces the camera ≈ 180° + viewing angle), so the
     *   shooter offset got rotated by the viewing angle — producing
     *   a lateral error that flipped sign on opposite sides of the hub.
     *
     *   NEW: SHOOTER_TO_CAMERA.plus(cameraToTag).plus(tagToHub)
     *
     *   SHOOTER_TO_CAMERA has ZERO rotation, so .plus(cameraToTag)
     *   is pure vector addition — no viewing-angle contamination.
     *   Then cameraToTag.plus(tagToHub) correctly rotates the
     *   tag-frame hub offset into the robot frame.
     * </pre>
     */
    public Transform3d getHubRelativeLocation() {
        Transform3d hub9 = null;
        Transform3d hub10 = null; // prefer
        Transform3d hub25 = null;
        Transform3d hub26 = null; // prefer

        for (final PhotonPipelineResult result : results) {
            for (final PhotonTrackedTarget target : result.getTargets()) {
                if (target.fiducialId != HUB_OFF_CENTER_RIGHT_RED_TAG && target.fiducialId != HUB_CENTER_RED_TAG &&
                        target.fiducialId != HUB_CENTER_BLUE_TAG && target.fiducialId != HUB_OFF_CENTER_LEFT_BLUE_TAG)
                    continue;

                if (target.fiducialId == HUB_OFF_CENTER_RIGHT_RED_TAG) {
                    hub9 = target.getBestCameraToTarget();
                    continue;
                }
                if (target.fiducialId == HUB_CENTER_RED_TAG) {
                    hub10 = target.getBestCameraToTarget();
                    continue;
                }
                if (target.fiducialId == HUB_OFF_CENTER_LEFT_BLUE_TAG) {
                    hub25 = target.getBestCameraToTarget();
                    continue;
                }
                if (target.fiducialId == HUB_CENTER_BLUE_TAG) {
                    hub26 = target.getBestCameraToTarget();
                }
            }
        }

        // Select which tag to use and its corresponding hub offset.
        // Center tags preferred (no lateral offset error).
        // Only one hub is ever visible — single chain is sufficient.
        Transform3d cameraToTag = null;
        Transform3d tagToHubOffset = null;

        if (hub10 != null) {
            cameraToTag = hub10;
            tagToHubOffset = TAG10_TO_HUB_CENTER_OFFSET;
        } else if (hub26 != null) {
            cameraToTag = hub26;
            tagToHubOffset = TAG26_TO_HUB_CENTER_OFFSET;
        } else if (hub9 != null) {
            cameraToTag = hub9;
            tagToHubOffset = TAG9_TO_HUB_CENTER_OFFSET;
        } else if (hub25 != null) {
            cameraToTag = hub25;
            tagToHubOffset = TAG25_TO_HUB_CENTER_OFFSET;
        }
        if (cameraToTag == null) {
            return null;
        }

        // ── Correct frame chain: Shooter → Camera → Tag → Hub Center ──
        //
        // SHOOTER_TO_CAMERA (no rotation) .plus(cameraToTag):
        // translation = shooterToCam + I·camToTag = simple addition ✓
        // rotation = I · R_camToTag = R_camToTag
        //
        // .plus(tagToHubOffset):
        // translation += R_camToTag · tagToHub (tag-frame → robot-frame) ✓
        //
        // Result: shooter-to-hub displacement in robot frame — exactly
        // what the ballistic solver expects.
        return SHOOTER_TO_CAMERA_OFFSET
                .plus(cameraToTag)
                .plus(tagToHubOffset);
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

            Vector<N3> dynamicStdDevs;
            int tagCount = result.getTargets().size();
            double avgDist = pose.estimatedPose.getTranslation().getNorm();
            if (tagCount >= 2) {
                // Multi-tag: geometry resolves ambiguity, trust XY well.
                // Heading is better than single-tag but Pigeon is still superior.
                if (avgDist < 3.0) {
                    dynamicStdDevs = VecBuilder.fill(0.1, 0.1, 0.4);
                } else {
                    dynamicStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
                }
            } else {
                // Single-tag: XY is decent at close range, degrades with distance.
                // Heading is unreliable — let Pigeon handle it entirely.
                if (avgDist < 3.0) {
                    dynamicStdDevs = VecBuilder.fill(0.3, 0.3, 999.0);
                } else if (avgDist < 5.0) {
                    dynamicStdDevs = VecBuilder.fill(0.5, 0.5, 999.0);
                } else {
                    dynamicStdDevs = VecBuilder.fill(1.0, 1.0, 999.0);
                }
            }

            final VisionMeasurement measurement = new VisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    result.getTimestampSeconds(),
                    dynamicStdDevs);

            poseUpdator.accept(measurement);
        }
    }
}