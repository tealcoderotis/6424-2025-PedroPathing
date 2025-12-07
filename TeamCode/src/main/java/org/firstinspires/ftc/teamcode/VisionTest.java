package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.LimelightPoseCorrector;

@TeleOp(name = "Vision Test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        LimelightPoseCorrector poseCorrector = new LimelightPoseCorrector(hardwareMap);
        follower.setStartingPose(new Pose(72, 120, Math.toRadians(90)));
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Pedro Pathing x", follower.getPose().getX());
            telemetry.addData("Pedro Pathing y", follower.getPose().getY());
            telemetry.addData("Pedro Pathing heading", follower.getPose().getHeading());
            Pose limelightPose = poseCorrector.getLimelightPose();
            if (limelightPose != null) {
                telemetry.addData("Limelight x", limelightPose.getX());
                telemetry.addData("Limelight y", limelightPose.getY());
                telemetry.addData("Limelight heading", limelightPose.getHeading());
            }
            else {
                telemetry.addData("Limelight pose", "Unknown");
            }
            if (gamepad1.aWasPressed()) {
                follower.setPose(limelightPose);
            }
            telemetry.update();
            follower.update();
        }
    }
}
