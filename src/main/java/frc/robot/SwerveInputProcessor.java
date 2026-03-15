package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveInputProcessor {
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);

    public Translation2d getProcessedTranslation(CommandXboxController controller) {
        double rawX = -controller.getLeftY();
        double rawY = -controller.getLeftX();

        Vector<N2> inputVector = VecBuilder.fill(rawX, rawY);
        Vector<N2> curvedVector = MathUtil.copyDirectionPow(inputVector, 2.0, 1);

        double finalX = xLimiter.calculate(curvedVector.get(0, 0));
        double finalY = yLimiter.calculate(curvedVector.get(1, 0));

        Translation2d finalTrans = new Translation2d(finalX, finalY);
        return finalTrans;
    }
}
