package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class Globals {
    //Communication between auto and teleop
    public static Alliance alliance = Alliance.UNKNOWN;
    public static Pose endingPose = null;
    public static void resetGlobals() {
        alliance = Alliance.UNKNOWN;
        endingPose = null;
    }

    //Constants
    public static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(300, 0, 0, 10, MotorControlAlgorithm.PIDF);
    public static final double SHOOTER_VELOCITY = 1050;
    public static final double SHOOTER_FAR_VELOCITY = 1875;
    public static final RevHubOrientationOnRobot IMU_ORIENTATION = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
    );
    public static final double INTAKE_SPEED = 0.75;
    public static final double FEEDER_INTAKE_VELOCITY = 573;
    public static final double FEEDER_IDLE_VELOCITY = 200;
    public static final double FEEDER_LAUNCH_VELOCITY = 573;
    public static final double FEEDER_BACK_VELOCITY = -200;
    public static final double SHOOTER_BACK_VELOCITY = 375;
    public static final int INTAKE_BACK_TIME = 0;
    public static final double VELOCITY_TOLERANCE = 50;
    public static final int REV_TIME = 1500;
    public static final int MAX_INTAKE_TIME = 2000;
    public static final int MAX_GATE_INTAKE_TIME = 1000;
}
