package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "JustLocalization")
public class JustLocalization extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D pose = llResult.getBotpose_MT2();
            telemetry.addData("Pose3D position = ", pose.toString());
            Pose2D bot2d = new Pose2D(pose.getPosition().unit, pose.getPosition().x, pose.getPosition().y, AngleUnit.RADIANS, pose.getOrientation().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Pose2D position = ", bot2d.toString());
            Pose botPose = PoseConverter.pose2DToPose(bot2d, InvertedFTCCoordinates.INSTANCE);
            telemetry.addData("x = ", (botPose.getY()-botPose.getX())/Math.sqrt(2)-61);
            telemetry.addData("y = ", (-botPose.getX()-botPose.getY())/Math.sqrt(2)+13.5);
        }
    }
}