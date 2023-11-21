package frc.robot.Utils.Chessy;


public interface State<S> extends CheesyInterpolable<S>, CheesyCSVWritable {
    double distance(final S other);

    S add(S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
