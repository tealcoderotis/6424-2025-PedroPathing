package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

@Configurable
@Disabled
@TeleOp
public class TestTeleop extends LinearOpMode {
    private Follower follower;
    private DcMotor shooter;
    private DcMotor intake;
    private Poses poses;
    private boolean autonomousDrive;

    @Override
    public void runOpMode() {
        poses = new Poses();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poses.BlueShooter);
        autonomousDrive = false;
        intake = (DcMotor)hardwareMap.get("indexMotor");
        shooter = (DcMotor)hardwareMap.get("shooterMotor");
        DcMotor rightFrontDrive = (DcMotor)hardwareMap.get("rightFrontDrive");
        DcMotor rightBackDrive = (DcMotor)hardwareMap.get("rightBackDrive");
        DcMotor leftFrontDrive = (DcMotor)hardwareMap.get("leftFrontDrive");
        DcMotor leftBackDrive = (DcMotor)hardwareMap.get("leftBackDrive");
        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            //drivebase
            if (!follower.isBusy() && autonomousDrive) {
                autonomousDrive = false;
                follower.startTeleopDrive();
            }
            if (!autonomousDrive) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            }
            else {
                //cancel automated movement upon joystick movement
                if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                    follower.breakFollowing();
                    autonomousDrive = false;
                    follower.startTeleopDrive();
                    follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
                }
            }
            if (gamepad1.aWasPressed()) {
                //moves to parking positon
                follower.breakFollowing();
                follower.followPath(createPathToPose(poses.BluePark));
                autonomousDrive = true;
            }
            if (gamepad1.yWasPressed()) {
                //moves to lever
                follower.breakFollowing();
                follower.followPath(createPathToPose(poses.BlueLever));
                autonomousDrive = true;
            }
            if (gamepad1.xWasPressed()) {
                //moves to intake
                follower.breakFollowing();
                follower.followPath(createPathToPose(poses.BlueIntake));
                autonomousDrive = true;
            }
            if (gamepad1.bWasPressed()) {
                //moves to shooter
                follower.breakFollowing();
                follower.followPath(createPathToPose(poses.BlueShooter));
                telemetry.addLine("b was pressed");
                autonomousDrive = true;
            }
            follower.update();
            //intake
            if (gamepad2.b) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }
            //shooter
            if (gamepad2.a) {
                shooter.setPower(1);
            }
            else {
                shooter.setPower(0);
            }
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("leftFrontDrive", leftFrontDrive.getPower());
            telemetry.addData("leftBackDrive", leftBackDrive.getPower());
            telemetry.addData("rightFrontDrive", rightFrontDrive.getPower());
            telemetry.addData("rightBackDrive", rightBackDrive.getPower());
            telemetry.addData("Indexer Encoder Position", intake.getCurrentPosition());
            telemetry.addData("Shooter Encoder Position", shooter.getCurrentPosition());
            telemetry.update();
        }
    }

    //derives a path starting at our current pose
    private PathChain createPathToPose(Pose pose) {
        return follower
                .pathBuilder()
                .addPath(new BezierLine(follower.getPose(), pose))
                .setLinearHeadingInterpolation(follower.getHeading(), pose.getHeading())
                .build();
    }
}
