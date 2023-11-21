package frc.robot.Utils.Chessy;

public interface CheesyIRotation2d<S> extends State<S> {
    CheesyRotation2d getRotation();

    S rotateBy(CheesyRotation2d other);

    S mirror();
}
