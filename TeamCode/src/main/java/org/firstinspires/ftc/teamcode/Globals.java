package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static final double SHOOTER_VELOCITY = 1400;
    public static final RevHubOrientationOnRobot IMU_ORIENTATION = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
    );
    public static final double INTAKE_SPEED = 0.25;
    public static final double FEEDER_INTAKE_VELOCITY = 1700;
    public static final double FEEDER_LAUNCH_VELOCITY = 1700;
    public static final double FEEDER_BACK_VELOCITY = -1700;
    public static final double SHOOTER_BACK_VELOCITY = 1700;
    public static final int INTAKE_BACK_TIME = 100;
    public static final double VELOCITY_TOLERANCE = 50;
    public static final String FEEDER_HARDWARE_MAP_NAME = "feeder";
    public static final String SHOOTER_1_HARDWARE_MAP_NAME = "launcher1";
    public static final String SHOOTER_2_HARDWARE_MAP_NAME = "launcher2";
    public static final String TURRET_ROTATE_HARDWARE_MAP_NAME = "turret";
    public static final double DEGREES_PER_TURRET_MOTOR_REVOLUTION = 10;
    public static final double TICKS_PER_TURRET_MOTOR_REVOLUTION = 384.5;
    public static final DcMotorSimple.Direction SHOOTER_1_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SHOOTER_2_DIRECTION = DcMotorSimple.Direction.FORWARD;
}
