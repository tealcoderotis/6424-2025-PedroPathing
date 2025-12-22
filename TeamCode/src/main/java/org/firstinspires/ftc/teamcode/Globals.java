package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class Globals {
    public static Alliance alliance = Alliance.UNKNOWN;
    public static Pose endingPose = null;
    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(300, 0, 0, 10, MotorControlAlgorithm.PIDF);
    public static final double SHOOTER_VELOCITY = 1400;
    public static final RevHubOrientationOnRobot IMU_ORIENTATION = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
    );

    public static final double FEEDER_INTAKE_VELOCITY = 2000;
    public static final double FEEDER_LAUNCH_VELOCITY = 1000;
    public static final double FEEDER_INTAKE_BACK_VELOCITY = -1100;
    public static final double SHOOTER_BACK_VELOCITY = 1100;
    public static final double VELOCITY_TOLERANCE = 50;

    public static void resetGlobals() {
        alliance = Alliance.UNKNOWN;
        endingPose = null;
    }
}
