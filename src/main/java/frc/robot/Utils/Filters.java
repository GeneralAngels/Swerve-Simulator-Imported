// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

/**
 * Add your docs here.
 */
public final class Filters {

    public static class AlphaFilter {
        double alpha = 0.5;
        double previousMeasurement = 0;

        public AlphaFilter(double alpha) {
            this.alpha = alpha;
        }

        public double calculate(double currentMeasurement) {
            double returnValue = alpha * previousMeasurement + (1 - alpha) * currentMeasurement;
            previousMeasurement = currentMeasurement;
            return returnValue;
        }
    }

    public double cubicSmooth(double value, double alpha) {
        return alpha * value + (1 - alpha) * Math.pow(value, 3);
    }

}
