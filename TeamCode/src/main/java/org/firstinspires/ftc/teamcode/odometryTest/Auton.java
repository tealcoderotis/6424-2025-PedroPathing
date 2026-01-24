package org.firstinspires.ftc.teamcode.odometryTest;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.ShooterIntake;
import org.firstinspires.ftc.teamcode.ShooterIntakeContinuous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LimelightPoseCorrector;

@Configurable
@Autonomous(name = "Odometry Testing Auto")
public class Auton extends LinearOpMode {
    private Follower follower;
    private int pathState;
    private Paths paths;
    private LimelightPoseCorrector poseCorrector;
    private boolean useLimelight = false;

    @Override
    public void runOpMode() {
        //initialization
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        paths = new Paths(follower);
        pathState = 0;
        DcMotor rightFrontDrive = (DcMotor) hardwareMap.get("rightFrontDrive");
        DcMotor rightBackDrive = (DcMotor) hardwareMap.get("rightBackDrive");
        DcMotor leftFrontDrive = (DcMotor) hardwareMap.get("leftFrontDrive");
        DcMotor leftBackDrive = (DcMotor) hardwareMap.get("leftBackDrive");
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
        waitForStart();
        if (useLimelight) {
            poseCorrector = new LimelightPoseCorrector(hardwareMap);
        }
        while (opModeIsActive()) {
            //update shooter and follower
            if (useLimelight) {
                follower.setPose(poseCorrector.correctPose(follower.getPose()));
            }
            follower.update();
            pathUpdate();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("leftFrontDrive", leftFrontDrive.getPower());
            telemetry.addData("leftBackDrive", leftBackDrive.getPower());
            telemetry.addData("rightFrontDrive", rightFrontDrive.getPower());
            telemetry.addData("rightBackDrive", rightBackDrive.getPower());
            telemetry.update();
        }
    }

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Up);
                    pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Right);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Down);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Left);
                    pathState = 0;
                }
                break;
        }
    }
}