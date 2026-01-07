package org.firstinspires.ftc.teamcode.util;

public class Regression {
    private static final double[] dataDistances = {50.996767, 69.574177, 84.84786, 102.93725, 118.13214};
    private static final int[] dataShooterSpeeds = {1360, 1420, 1530, 1650, 1740};

    public static double getVelocityForDistance(double distance) {
        double flywheelVelocity = 0;
        for (int i = 1; i < dataDistances.length; i++) {
            if (distance < dataDistances[i] && distance >= dataDistances[i-1]) {
                flywheelVelocity = dataShooterSpeeds[i] + (dataShooterSpeeds[i]-dataShooterSpeeds[i-1])/(dataDistances[i]-dataDistances[i-1]) * (distance - dataDistances[i]);
            }
        }
        return flywheelVelocity;
    }
}
