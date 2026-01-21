package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.ShooterMath;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TobyTele")
//@Disabled
public class TobyTele extends OpMode {
    ShooterMath shootermath;
    final double FEED_TIME_SECONDS = 0.75;
    final double STOP_SPEED = 0.0;

    final double LAUNCHER_IDLE_VELOCITY = 0;
    final double LAUNCHER_MAX_VELOCITY = 1950;
    final double LAUNCHER_MIN_VELOCITY = 1500;
    final double LAUNCHER_SPINUP_VELOCITY = 1200;
    final double FEEDER_INTAKE_VELOCITY = 1100;
    final double FEEDER_LAUNCH_VELOCITY = 1000;

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

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        SPIN_UP_FAR,
        LAUNCH,
        LAUNCH_FAR,
        LAUNCHING,
    }

    private LaunchState launchState;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        shootermath = new ShooterMath(telemetry);
        launchState = LaunchState.IDLE;
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
        }
        else {
            if (gamepad1.bWasPressed()) {
                //Red starting pose
                follower.setStartingPose(new Pose(125.200, 70.930, Math.toRadians(0)));
                alliance = Alliance.RED;
                telemetry.addLine("Red alliance");
                telemetry.update();
            } else if (gamepad1.xWasPressed()) {
                //Blue starting pose
                follower.setStartingPose(new Pose(46.892, 59.798, Math.toRadians(180)));
                alliance = Alliance.BLUE;
                telemetry.addLine("Blue alliance");
                telemetry.update();
            }
        }
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
            feeder.setVelocity(FEEDER_INTAKE_VELOCITY);
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
        }
        else {
            feeder.setPower(STOP_SPEED);
        }
        if (gamepad2.dpad_left) {
            double goalBallVelocity = 0;
            if (alliance == Alliance.RED) {
                goalBallVelocity = shootermath.findLateralVelocity(follower.getPose(), 144, 144);
            } else if (alliance == Alliance.BLUE) {
                goalBallVelocity = shootermath.findLateralVelocity(follower.getPose(), 0, 144);
            } else {
                telemetry.addData("Goal Ball Velocity", "UNKNOWN ALLIANCE");
            }
            telemetry.addData("Goal Ball Velocity", goalBallVelocity);
            double flywheelVelocity = shootermath.ballVelocityToFlywheel(goalBallVelocity);
            launcher.setVelocity(flywheelVelocity);
            telemetry.addData("Shooter Speed", flywheelVelocity);
        }

        launch(gamepad2.right_trigger >= 0.1);

        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
    }

    void mecanumDrive(double forward, double strafe, double rotate){

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

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case SPIN_UP_FAR:
                if (launcher.getVelocity() > LAUNCHER_MAX_VELOCITY) {
                    launchState = LaunchState.LAUNCH_FAR;
                }
                break;
            case LAUNCH:
                feeder.setDirection(DcMotor.Direction.FORWARD);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCH_FAR:
                feeder.setDirection(DcMotor.Direction.FORWARD);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    feeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}

