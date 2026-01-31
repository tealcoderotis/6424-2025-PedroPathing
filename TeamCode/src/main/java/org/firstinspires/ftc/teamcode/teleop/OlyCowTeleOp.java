package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Regression;
import org.firstinspires.ftc.teamcode.ShooterMath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseTrig;

import java.util.List;

@Configurable
@TeleOp(name = "OlyCowTeleOp (Android Studio Version)")
@Disabled
public class OlyCowTeleOp extends OpMode {
    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = 0.0;
    //final double HALF_SPEED = 0.5;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_MAX_VELOCITY = 1575;
    final double LAUNCHER_MIN_VELOCITY = 1000;
    final double LAUNCHER_SPINUP_VELOCITY = 1100;
    final double MINIMUM_ADJUSTMENT = 0.05;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx feeder = null;
    private Limelight3A limelight;
    private Follower follower;

    ElapsedTime feederTimer = new ElapsedTime();

    private Alliance alliance = Alliance.UNKNOWN;

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
    boolean lockOn = false; //Whether to use a PIDF to lock on the goal.
    boolean slowMode = false;
    double pCorrection = 0;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

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

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        feeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Globals.SHOOTER_PIDF);

        follower = Constants.createFollower(hardwareMap);
        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(5);*/
        telemetry.setMsTransmissionInterval(11);
        /*limelight.pipelineSwitch(0);
        limelight.start();*/

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
            } else {
                telemetry.addLine("ALLIANCE UNKNOWN");
            }
        }
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        follower.update();
    }

    @Override
    public void start() {
        //limelight.start();
        follower.startTeleOpDrive(false);
    }

    @Override
    public void loop() {
        //LLResult result = limelight.getLatestResult();
        if (!lockOn) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        } else {
            if (slowMode) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/3.5);
            } /*else if (result.isValid()) {
                //PIDF to control rotation rate while locked on the goal
                double P_gain = 0.05;
                double headingCorrectionNeeded = result.getTx(); //Degrees
                telemetry.addData("Heading correction needed", headingCorrectionNeeded);
                double D_gain = 0.007;
                double dCorrection = headingCorrectionNeeded-pCorrection;
                double rotate = P_gain * headingCorrectionNeeded + D_gain * dCorrection;
                if (Math.abs(rotate) > 0.5) {
                    rotate = rotate / Math.abs(rotate);
                }
                pCorrection = headingCorrectionNeeded;
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, rotate, false);
                if (Math.abs(headingCorrectionNeeded)<1) {
                    slowMode = true;
                }
            }*/ else {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/1.5);
            }
        }
        if (gamepad1.left_bumper) {
            lockOn = true;
        }

        if (gamepad2.x){
            launcher.setDirection(DcMotor.Direction.REVERSE);
            launcher.setPower(0.25);
            feeder.setDirection(DcMotor.Direction.REVERSE);
            feeder.setPower(FULL_SPEED);
        }

        if (gamepad2.a) {
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setPower(FULL_SPEED);
            launcher.setDirection(DcMotor.Direction.REVERSE);
            launcher.setPower(0.25);
        }
        else {
            feeder.setPower(STOP_SPEED);
        }
        if (gamepad1.a) {
            feeder.setVelocity(FULL_SPEED);
        }

        if (gamepad2.b) {
            launcher.setVelocity(STOP_SPEED);
            feeder.setPower(STOP_SPEED);
            lockOn = false;
            slowMode = false;
        }
        if (gamepad2.y) {
            launcher.setVelocity(LAUNCHER_SPINUP_VELOCITY);
        } else {
            launcher.setVelocity(STOP_SPEED);
        }
        if (gamepad2.dpad_up) {
            launcher.setVelocity(LAUNCHER_MAX_VELOCITY);
        }
        if (gamepad2.dpad_left) {
            double flywheelVelocity = LAUNCHER_MIN_VELOCITY;
            /*if (result.isValid()) {
                double ty = result.getTy();
                flywheelVelocity = 16.03396*ty+1469.24123;
                if (flywheelVelocity == 0) {
                    telemetry.addLine("Data not available for current distance!");
                }
            }*/
            launcher.setVelocity(flywheelVelocity);
            telemetry.addData("Shooter Speed", flywheelVelocity);
        }
        if (gamepad2.dpad_down) {
            launcher.setVelocity(LAUNCHER_MIN_VELOCITY);
        }

        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());

        if (alliance != Alliance.UNKNOWN) {
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        }
        telemetry.addData("Status", "Initialized");
        follower.update();
    }

    @Override
    public void stop() {
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