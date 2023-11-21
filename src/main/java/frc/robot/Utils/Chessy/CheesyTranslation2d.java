package frc.robot.Utils.Chessy;


import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
 */
public class CheesyTranslation2d implements CheesyITranslation2d<CheesyTranslation2d> {
    protected static final CheesyTranslation2d kIdentity = new CheesyTranslation2d();

    public static CheesyTranslation2d identity() {
        return kIdentity;
    }

    protected final double x_;
    protected final double y_;

    public CheesyTranslation2d() {
        x_ = 0;
        y_ = 0;
    }

    public CheesyTranslation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public CheesyTranslation2d(final CheesyTranslation2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public CheesyTranslation2d(final CheesyTranslation2d start, final CheesyTranslation2d end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    public CheesyTranslation2d(final edu.wpi.first.math.geometry.Translation2d other) {
        x_= other.getX();
        y_ = other.getY();
    }


    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm() {
        return Math.hypot(x_, y_);
    }

    public double norm2() {
        return x_ * x_ + y_ * y_;
    }

    public double x() {
        return x_;
    }

    public double y() {
        return y_;
    }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public CheesyTranslation2d translateBy(final CheesyTranslation2d other) {
        return new CheesyTranslation2d(x_ + other.x_, y_ + other.y_);
    }

    public CheesyTranslation2d plus(CheesyTranslation2d other) {
        return new CheesyTranslation2d(x_ + other.x(), y_ + other.y());
    }

    public CheesyTranslation2d minus(CheesyTranslation2d other) {
        return new CheesyTranslation2d(x_ - other.x(), y_ - other.y());
    }

    public CheesyTranslation2d unaryMinus() {
        return new CheesyTranslation2d(-x_, -y_);
    }

    public CheesyTranslation2d times(double scalar) {
        return new CheesyTranslation2d(x_ * scalar, y_ * scalar);
    }

    /**
     * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public CheesyTranslation2d rotateBy(final CheesyRotation2d rotation) {
        return new CheesyTranslation2d(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }

    public CheesyRotation2d direction() {
        return new CheesyRotation2d(x_, y_, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public CheesyTranslation2d inverse() {
        return new CheesyTranslation2d(-x_, -y_);
    }

    @Override
    public CheesyTranslation2d interpolate(final CheesyTranslation2d other, double x) {
        if (x <= 0) {
            return new CheesyTranslation2d(this);
        } else if (x >= 1) {
            return new CheesyTranslation2d(other);
        }
        return extrapolate(other, x);
    }

    public CheesyTranslation2d extrapolate(final CheesyTranslation2d other, double x) {
        return new CheesyTranslation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }

    public CheesyTranslation2d scale(double s) {
        return new CheesyTranslation2d(x_ * s, y_ * s);
    }

    public boolean epsilonEquals(final CheesyTranslation2d other, double epsilon) {
        return CheesyUtil.epsilonEquals(x(), other.x(), epsilon) && CheesyUtil.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat format = new DecimalFormat("#0.000");
        return "(" + format.format(x_) + "," + format.format(y_) + ")";
    }

    @Override
    public String toCSV() {
        final DecimalFormat format = new DecimalFormat("#0.000");
        return format.format(x_) + "," + format.format(y_);
    }

    public static double dot(final CheesyTranslation2d a, final CheesyTranslation2d b) {
        return a.x_ * b.x_ + a.y_ * b.y_;
    }

    public static CheesyRotation2d getAngle(final CheesyTranslation2d a, final CheesyTranslation2d b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new CheesyRotation2d();
        }
        return CheesyRotation2d.fromRadians(Math.acos(CheesyUtil.limit(cos_angle, 1.0)));
    }

    public static double cross(final CheesyTranslation2d a, final CheesyTranslation2d b) {
        return a.x_ * b.y_ - a.y_ * b.x_;
    }

    @Override
    public double distance(final CheesyTranslation2d other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public CheesyTranslation2d add(CheesyTranslation2d other) {
        return this.translateBy(other);
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof CheesyTranslation2d)) {
            return false;
        }

        return distance((CheesyTranslation2d) other) < CheesyUtil.kEpsilon;
    }

    @Override
    public CheesyTranslation2d getTranslation() {
        return this;
    }
}
