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
    private final static byte HUB_OFF_CENTER_LEFT_TAG = 9;
    private final static byte HUB_CENTER_TAG = 10;
    private final static Transform3d SHOOTER_TO_CAMERA_OFFSET = new Transform3d(
            // x
            Distance.ofRelativeUnits(-27.5, Inches),
            // y
            Distance.ofRelativeUnits(-2, Inches),
            // z
            Distance.ofRelativeUnits(3.70, Inches),
            new Rotation3d(
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees),
                    Angle.ofRelativeUnits(0, Degrees)));
    private static final Transform3d ROBOT_TO_CAMERA_OFFSET = new Transform3d(
            // x
            Distance.ofBaseUnits(25, Inches),
            // y
            Distance.ofBaseUnits(-2, Inches),
            // z
            Distance.ofBaseUnits(23.5, Inches),
            new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees)));

    private SendableChooser<AprilTagFields> fieldChooser = new SendableChooser<>();
    private AprilTagFields lastField = null;
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
        fieldChooser.setDefaultOption("Welded", AprilTagFields.k2026RebuiltWelded);
        fieldChooser.addOption("AndyMark", AprilTagFields.k2026RebuiltAndymark);
        SmartDashboard.putData(fieldChooser);
        this.camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        AprilTagFields initial = fieldChooser.getSelected();
        if (initial == null) {
            initial = AprilTagFields.k2026RebuiltWelded;
        }
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(initial),
                ROBOT_TO_CAMERA_OFFSET);
    }

    // -----------------------------------------------------------------------------------------------------------------
    /**
     * {@summary}
     * This needs to be run <b>EVERY</b> frame of the Robot's periodic function.
     * 
     * @apiNote Don't ever call this function unless you know what you are doing.
     */
    public void updateFrames() {
        // System.out.println("Updating camera frames.");
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
        Transform3d hub9 = null;
        Transform3d hub10 = null;

        for (final PhotonPipelineResult result : results) {
            for (final PhotonTrackedTarget target : result.getTargets()) {
                if (target.fiducialId != HUB_OFF_CENTER_LEFT_TAG && target.fiducialId != HUB_CENTER_TAG)
                    continue;

                if (target.fiducialId == HUB_OFF_CENTER_LEFT_TAG) {
                    hub9 = target.getBestCameraToTarget();
                    continue;
                }
                if (target.fiducialId == HUB_CENTER_TAG) {
                    hub10 = target.getBestCameraToTarget();
                }
            }
        }

        Transform3d hub = null;
        /*
         * The center of the hub is deeper than the tags in the x direction: account for
         * that.
         * 
         * Also, set the "hub" being pointed at from what tag and offset our measurement
         * to account for what tag being used.
         */
        Transform3d tagToHubCenterOffset = null;
        if (hub9 != null && hub10 == null) {
            // System.out.println("Hub is tag 9.");
            tagToHubCenterOffset = new Transform3d(
                    // x
                    Distance.ofRelativeUnits(-23.5, Inches),
                    // y
                    Distance.ofRelativeUnits(15, Inches),
                    // z
                    Distance.ofRelativeUnits(27.5, Inches),
                    new Rotation3d(
                            Angle.ofRelativeUnits(0, Degrees),
                            Angle.ofRelativeUnits(0, Degrees),
                            Angle.ofRelativeUnits(0, Degrees)));
            hub = hub9;
        } else if ((hub9 == null || hub9 != null) && hub10 != null) {
            // System.out.println("Hub is tag 10.");
            tagToHubCenterOffset = new Transform3d(
                    // x
                    Distance.ofRelativeUnits(-23.5, Inches),
                    // y
                    Distance.ofRelativeUnits(0, Inches),
                    // z
                    Distance.ofRelativeUnits(27.5, Inches),
                    new Rotation3d(
                            Angle.ofRelativeUnits(0, Degrees),
                            Angle.ofRelativeUnits(0, Degrees),
                            Angle.ofRelativeUnits(0, Degrees)));
            hub = hub10;
        }

        if (hub != null) {
            /*
             * System.out.println(String.format("hub tag (x=%f,y=%f,z=%f)",
             * hub.getMeasureX().in(Inches),
             * hub.getMeasureY().in(Inches),
             * hub.getMeasureZ().in(Inches)));
             */

            hub = hub.plus(SHOOTER_TO_CAMERA_OFFSET);
            /*
             * System.out.println(String.format("hub shooter (x=%f,y=%f,z=%f)",
             * hub.getMeasureX().in(Inches),
             * hub.getMeasureY().in(Inches),
             * hub.getMeasureZ().in(Inches)));
             */
            hub = hub.plus(tagToHubCenterOffset);
            /*
             * System.out.println(String.format("hub dist (x=%f,y=%f,z=%f)",
             * hub.getMeasureX().in(Inches),
             * hub.getMeasureY().in(Inches),
             * hub.getMeasureZ().in(Inches)));
             */
        }

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
