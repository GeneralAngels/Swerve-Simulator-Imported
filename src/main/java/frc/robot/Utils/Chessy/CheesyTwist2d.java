package frc.robot.Utils.Chessy;

import java.text.DecimalFormat;
import java.util.Optional;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 * <p>
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class CheesyTwist2d implements CheesyInterpolable<CheesyTwist2d>, ICourse2d<CheesyTwist2d> {
    protected static final CheesyTwist2d kIdentity = new CheesyTwist2d(0.0, 0.0, 0.0);

    public static CheesyTwist2d identity() {
        return kIdentity;
    }

    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!

    public CheesyTwist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public CheesyTwist2d scaled(double scale) {
        return new CheesyTwist2d(dx * scale, dy * scale, dtheta * scale);
    }

    public CheesyTwist2d mirror() {
        return new CheesyTwist2d(dx, -dy, -dtheta);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double norm2() {
        return dx * dx + dy * dy;
    }

    public boolean hasTranslation() {
        return norm2() > CheesyUtil.kEpsilon;
    }

    // Return a vector in the local direction of motion
    @Override
    public Optional<CheesyRotation2d> getCourse() {
        if (hasTranslation()) {
            return Optional.of(new CheesyRotation2d(dx, dy, true));
        } else {
            return Optional.empty();
        }
    }

    // Commented out to avoid confusion since we use Twists to refer to various things, some of which have meaningful curvatures and others don't.
    /*public double curvature() {
        if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return dtheta / norm();
    }*/

    public boolean epsilonEquals(final CheesyTwist2d other, double epsilon) {
        return CheesyUtil.epsilonEquals(dx, other.dx, epsilon) &&
                CheesyUtil.epsilonEquals(dy, other.dy, epsilon) &&
                CheesyUtil.epsilonEquals(dtheta, other.dtheta, epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }

    @Override
    public CheesyTwist2d interpolate(CheesyTwist2d other, double x) {
        return new CheesyTwist2d(CheesyUtil.interpolate(dx, other.dx, x),
                CheesyUtil.interpolate(dy, other.dy, x),
                CheesyUtil.interpolate(dtheta, other.dtheta, x));
    }
}