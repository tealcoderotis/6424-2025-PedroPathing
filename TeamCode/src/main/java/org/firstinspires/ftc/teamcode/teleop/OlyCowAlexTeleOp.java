package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.util.Regression;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "OlyCowAlexTeleOp")
//@Disabled
public class OlyCowAlexTeleOp extends OpMode {
    final double FEED_TIME_SECONDS = 0.75;
    final double STOP_SPEED = 0.0;

    final double LAUNCHER_IDLE_VELOCITY = 0;
    final double LAUNCHER_MAX_VELOCITY = 1950;
    final double LAUNCHER_MIN_VELOCITY = 1500;
    final double LAUNCHER_SPINUP_VELOCITY = 1200;
    final double FEEDER_INTAKE_VELOCITY = 1700;
    final double FEEDER_LAUNCH_VELOCITY = 1700;
    final double FEEDER_REVERSE_VELOCITY = 900;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx feeder = null;
    private Follower follower;
    boolean lockOn = false;
    ElapsedTime feederTimer = new ElapsedTime();
    private Alliance alliance = Alliance.UNKNOWN;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        feeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (Globals.alliance != Alliance.UNKNOWN) {
            alliance = Globals.alliance;
            follower.setStartingPose(Globals.endingPose.copy());
        } else {
            if (gamepad1.bWasPressed()) {
                //Red starting pose
                follower.setStartingPose(new Pose(97.108, 59.579, Math.toRadians(0)));
                alliance = Alliance.RED;
            } else if (gamepad1.xWasPressed()) {
                //Blue starting pose
                follower.setStartingPose(new Pose(46.892, 59.798, Math.toRadians(180)));
                alliance = Alliance.BLUE;
            }
        }
        telemetry.addData("alliance", alliance.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(false);
    }

    @Override
    public void loop() {
        if (!lockOn) {
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            double rotate = 0;
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
        }

        if (gamepad2.a) {
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setVelocity(FEEDER_INTAKE_VELOCITY);
            if (launcher.getVelocity() > LAUNCHER_IDLE_VELOCITY) {
                feeder.setDirection(DcMotor.Direction.FORWARD);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
            }
        }
        if (gamepad2.x) {
            launcher.setDirection(DcMotor.Direction.REVERSE);
            telemetry.addData("Goal Ball Velocity", LAUNCHER_IDLE_VELOCITY);
            launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
            telemetry.addData("Shooter Speed", LAUNCHER_IDLE_VELOCITY);
            feeder.setDirection(DcMotor.Direction.REVERSE);
            feeder.setVelocity(FEEDER_REVERSE_VELOCITY);
        }

        if (gamepad2.b) {
            telemetry.addData("Goal Ball Velocity", LAUNCHER_IDLE_VELOCITY);
            launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
            telemetry.addData("Shooter Speed", LAUNCHER_IDLE_VELOCITY);
        }
        if (gamepad2.y) {
            launcher.setVelocity(LAUNCHER_SPINUP_VELOCITY);
        }
        if (gamepad2.dpad_up) {
            telemetry.addData("Goal Ball Velocity", "MAXIMUM");
            launcher.setVelocity(LAUNCHER_MAX_VELOCITY);
            telemetry.addData("Shooter Speed", "MAXIMUM");
        }

        if (gamepad2.dpad_down) {
            telemetry.addData("Goal Ball Velocity", "MINIMUM");
            launcher.setVelocity(LAUNCHER_MIN_VELOCITY);
            telemetry.addData("Shooter Speed", "MINIMUM");
        } else {
            feeder.setPower(STOP_SPEED);
        }
        if (gamepad2.dpad_left) {
            double dist = 0;
            if (alliance == Alliance.RED) {
                dist = Math.sqrt(Math.pow(144 - follower.getPose().getX(), 2) + Math.pow(144 - follower.getPose().getY(), 2));
            } else if (alliance == Alliance.BLUE) {
                dist = Math.sqrt(Math.pow(0 - follower.getPose().getX(), 2) + Math.pow(144 - follower.getPose().getY(), 2));
            } else {
                telemetry.addData("Goal Ball Velocity", "UNKNOWN ALLIANCE");
            }
            double flywheelVelocity = Regression.getVelocityForDistance(dist);
            if (flywheelVelocity == 0) {
                telemetry.addLine("Data not available for current distance!");
            }
            launcher.setVelocity(flywheelVelocity);
            telemetry.addData("Shooter Speed", flywheelVelocity);
        }
        if (gamepad2.right_bumper) {
            follower.setPose((new Pose(110.36335877862595, 134.10687022900763, 0)));
        }

        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("dist", Math.sqrt(Math.pow(144 - follower.getPose().getX(), 2) + Math.pow(144 - follower.getPose().getY(), 2)));
        follower.update();
    }

    void mecanumDrive(double forward, double strafe, double rotate) {

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }
}