package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class AimCamera {
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
    public Transform3d getHubRelativeLocation() throws RuntimeException {
        throw new RuntimeException("Not implemented yet.");
    }
}
