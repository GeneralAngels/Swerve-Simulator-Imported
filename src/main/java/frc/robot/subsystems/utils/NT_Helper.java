package frc.robot.subsystems.utils;

import edu.wpi.first.networktables.*;

public class NT_Helper {

    public static DoubleSubscriber getDoubleSubscriber(NetworkTable table, String topic, double _default) {
        DoubleSubscriber subscriber = table.getDoubleTopic(topic).subscribe(_default);

        DoublePublisher publisher = table.getDoubleTopic(topic).publish();
        publisher.set(_default);


        return subscriber;
    }

    public static IntegerSubscriber getIntSubscriber(NetworkTable table, String topic, int _default) {
        IntegerSubscriber subscriber = table.getIntegerTopic(topic).subscribe(_default);

        IntegerPublisher publisher = table.getIntegerTopic(topic).publish();
         publisher.set(_default);


        return subscriber;
    }
}
