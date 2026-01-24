package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Disabled
@Autonomous
public class OdometryTestAuton extends LinearOpMode {
    private Follower follower;
    private PathChain path;

    @Override
    public void runOpMode() {
        DcMotor rightFrontDrive = (DcMotor)hardwareMap.get("rightFrontDrive");
        DcMotor rightBackDrive = (DcMotor)hardwareMap.get("rightBackDrive");
        DcMotor leftFrontDrive = (DcMotor)hardwareMap.get("leftFrontDrive");
        DcMotor leftBackDrive = (DcMotor)hardwareMap.get("leftBackDrive");
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.25);
        path = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(0, 0, Math.toRadians(0)), new Pose(24, 0, Math.toRadians(0)))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        waitForStart();
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.followPath(path, false);
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("leftFrontDrive", leftFrontDrive.getPower());
            telemetry.addData("leftBackDrive", leftBackDrive.getPower());
            telemetry.addData("rightFrontDrive", rightFrontDrive.getPower());
            telemetry.addData("rightBackDrive", rightBackDrive.getPower());
            telemetry.update();
        }
    }
}
