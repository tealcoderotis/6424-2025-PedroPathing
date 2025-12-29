package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "OlyCowAidenTeleOp (Android Studio Version)")
//@Disabled
public class OlyCowAidenTeleOp extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 2175;
    final double LAUNCHER_MIN_VELOCITY = 1450;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx shooterMotor = null;
    private DcMotorEx indexMotor = null;
    ElapsedTime feederTimer = new ElapsedTime();

    public double konst = 0.6;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        indexMotor = hardwareMap.get(DcMotorEx.class, "feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        shooterMotor.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.a) {
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setPower(0.3);
            indexMotor.setDirection(DcMotor.Direction.FORWARD);
            indexMotor.setPower(0.5);
        } else if (gamepad2.y) {
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setPower(0.4);
            indexMotor.setDirection(DcMotor.Direction.REVERSE);
            indexMotor.setPower(0.3);
        } else if (gamepad2.b) {
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
            //shooterMotor.setPower(0.55);
            if (gamepad2.dpad_up) {
                indexMotor.setDirection(DcMotor.Direction.FORWARD);
                indexMotor.setPower(0.4);
            } else if (gamepad2.dpad_down) {
                indexMotor.setDirection(DcMotor.Direction.REVERSE);
                indexMotor.setPower(0.4);
            } else {
                indexMotor.setDirection(DcMotor.Direction.REVERSE);
                indexMotor.setPower(0);
            }
        }
        else if (gamepad2.x) {
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            shooterMotor.setVelocity(LAUNCHER_MIN_VELOCITY);
            //shooterMotor.setPower(0.8);
            shooterMotor.setPower(0.5);
            if (gamepad2.dpad_up) {
                indexMotor.setDirection(DcMotor.Direction.FORWARD);
                indexMotor.setPower(0.4);
            } else if (gamepad2.dpad_down) {
                indexMotor.setDirection(DcMotor.Direction.REVERSE);
                indexMotor.setPower(0.4);
            } else {
                indexMotor.setDirection(DcMotor.Direction.REVERSE);
                indexMotor.setPower(0);
            }
        } else {
            shooterMotor.setPower(0);
            indexMotor.setPower(0);
        }

        telemetry.addData("State", launchState);
        telemetry.addData("motorVelocity", shooterMotor.getVelocity());
        telemetry.addData("indexPower", shooterMotor.getPower());

    }

    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){
        if (gamepad1.a) {konst = 0.3;}
        else {konst = 0.6;}

        double denominator = (Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1)) * konst;

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
                shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (shooterMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}


