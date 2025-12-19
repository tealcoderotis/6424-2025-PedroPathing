package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//PedroPathing Constants
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15) //TODO Replace mass with accurate value. Mass is in kilograms
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.002, 0.046));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(56.7362800788)
            .yVelocity(42.8251357520);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("leftFrontDrive")
            .strafeEncoder_HardwareMapName("leftBackDrive")
            .forwardPodY(-5.875)
            .strafePodX(-7.375)
            .forwardTicksToInches(0.002976417016479054)
            .strafeTicksToInches(0.0029601869631298407)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );

    public static ThreeWheelIMUConstants threeWheelLocalizerConstants = new ThreeWheelIMUConstants()
            .leftEncoder_HardwareMapName("leftFrontDrive")
            .rightEncoder_HardwareMapName("rightFrontDrive")
            .strafeEncoder_HardwareMapName("leftBackDrive")
            .leftPodY(6)
            .rightPodY(-6.125)
            .strafePodX(-9)
            .forwardTicksToInches(0.002978295301409603)
            .strafeTicksToInches(0.002969057822227131)
            .turnTicksToInches(0.0029571787252668353)
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(threeWheelLocalizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
