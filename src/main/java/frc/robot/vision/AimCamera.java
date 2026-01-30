package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class AimCamera {
    // =================================================================================================================
    // Constants
    // =================================================================================================================
    private final static byte HUB_CENTER_TAG = 9;
    private final static byte HUB_OFF_CENTER_RIGHT_TAG = 10;
    private final static Transform3d SHOOTER_TO_CAMERA_OFFSET = new Transform3d(
            Distance.ofBaseUnits(5, Inches),
            Distance.ofBaseUnits(0, Inches),
            Distance.ofBaseUnits(-12, Inches),
            new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees)));

    // =================================================================================================================
    // Systems
    // =================================================================================================================
    private final PhotonCamera camera;

    public AimCamera() {
        this.camera = new PhotonCamera("photonvision");
    }

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

        for (final PhotonPipelineResult result : camera.getAllUnreadResults()) {
            for (final PhotonTrackedTarget target : result.getTargets()) {
                if (target.fiducialId != HUB_CENTER_TAG || target.fiducialId != HUB_OFF_CENTER_RIGHT_TAG)
                    continue;
                hub = target.getBestCameraToTarget();
            }
        }

        if (hub != null)
            hub.plus(SHOOTER_TO_CAMERA_OFFSET);

        return hub;
    }
}
