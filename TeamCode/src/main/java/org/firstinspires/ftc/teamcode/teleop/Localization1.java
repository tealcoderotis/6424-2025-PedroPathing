package org.firstinspires.ftc.teamcode.teleop;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
@Configurable
@TeleOp(name = "Localization1")
public class Localization1 extends OpMode {
    private Limelight3A limelight;
    private Follower follower;
    private IMU imu;
    private Alliance alliance = Alliance.UNKNOWN;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(5);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // only april tag 22
        limelight.start();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));//Initialize while facing right? maybe forward? Not 45 degrees.

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop() {
        if (gamepad1.bWasPressed()) {
            //Red starting pose
            follower.setStartingPose(new Pose(125.200, 70.930, Math.toRadians(0)));
            alliance = Alliance.RED;
            limelight.pipelineSwitch(0); // only april tag 24
            telemetry.addLine("Red alliance, tag 24");
        } else if (gamepad1.xWasPressed()) {
            //Blue starting pose
            follower.setStartingPose(new Pose(46.892, 59.798, Math.toRadians(180)));
            alliance = Alliance.BLUE;
            limelight.pipelineSwitch(1); // only april tag 20
            telemetry.addLine("Blue alliance, tag 20");
        } else {
            telemetry.addLine("ALLIANCE UNKNOWN");
        }
        telemetry.update();
    }
    @Override
    public void start() {
        limelight.start();
    }
    public static Pose pose3DToPedroPose(Pose3D pose) {
        Pose2D pose2d = new Pose2D(pose.getPosition().unit, pose.getPosition().x, pose.getPosition().y, AngleUnit.RADIANS, pose.getOrientation().getYaw(AngleUnit.RADIANS));
        Pose ftcStandard = PoseConverter.pose2DToPose(pose2d, InvertedFTCCoordinates.INSTANCE);
        ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        return ftcStandard;
    }
    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result.isValid()){
            Pose3D botposemt1 = result.getBotpose();//mt1
            Pose3D botposemt2 = result.getBotpose_MT2();//mt2
            Pose posemt1 = pose3DToPedroPose(botposemt1);
            Pose posemt2 = pose3DToPedroPose(botposemt2);
            telemetry.addData("Megatag1 Pose", posemt1.toString());
            telemetry.addData("Megatag2 Pose", posemt2.toString());
        } else {
            telemetry.addLine("Result is not valid");
        }

    }
}
