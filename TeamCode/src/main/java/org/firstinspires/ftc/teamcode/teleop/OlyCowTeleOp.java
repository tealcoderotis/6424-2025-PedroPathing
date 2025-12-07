package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.ShooterMath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseTrig;

import java.util.List;

@TeleOp(name = "OlyCowTeleOp (Android Studio Version)")
//@Disabled
public class OlyCowTeleOp extends OpMode {
    private ShooterMath shootermath;
    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = 0.0;
    final double HALF_SPEED = 0.5;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_MAX_VELOCITY = 1575;
    final double LAUNCHER_MIN_VELOCITY = 1000;
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
    double trackingAngle;
    double currentShootVelocity = 1575;
    boolean overrideShootVelocity = false;
    boolean lockOn = false; //Whether to use a PIDF to lock on the goal.

    @Override
    public void init() {
        shootermath = new ShooterMath(telemetry);
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

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        follower = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

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
        limelight.start();
        follower.startTeleOpDrive(false);
    }

    @Override
    public void loop() {
        if (!lockOn) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
        } else {
            //PIDF to control rotation rate while locked on the goal
            int GoalX = 0; int GoalY=144;
            if(!overrideShootVelocity && alliance != Alliance.UNKNOWN) {
                if (alliance == Alliance.RED) {GoalX = 144;}
            }
            double P = 0.05;
            double headingCorrectionNeeded = shootermath.angleFromGoal(follower.getPose(), GoalX, GoalY); //Degrees
            telemetry.addData("Heading correction needed", headingCorrectionNeeded);
            double rotate = P*headingCorrectionNeeded;
            if (Math.abs(rotate) > 1) {
                rotate = rotate/Math.abs(rotate);
            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate, false);
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
        }
        else {
            feeder.setPower(STOP_SPEED);
        }

        if (gamepad2.b) {
            launcher.setVelocity(STOP_SPEED);
            feeder.setPower(STOP_SPEED);
        }
        if (gamepad2.y) {
            launcher.setVelocity(currentShootVelocity);
        }
        if (gamepad2.dpad_up) {
            currentShootVelocity = LAUNCHER_MAX_VELOCITY;
            overrideShootVelocity = true;
        }

        if (gamepad2.dpad_down) {
            currentShootVelocity = LAUNCHER_MIN_VELOCITY;
            overrideShootVelocity = true;
        }
        if (gamepad1.dpad_down) {
            lockOn = true;
        } else if (gamepad1.dpad_up) {
            lockOn = false;
        }
        if(!overrideShootVelocity && alliance != Alliance.UNKNOWN) {
            if (alliance == Alliance.RED) {
                telemetry.addData("Goal ball velocity", shootermath.findLateralVelocity(follower.getPose(), 144, 144));
                currentShootVelocity = shootermath.ballVelocityToFlywheel(shootermath.findLateralVelocity(follower.getPose(), 144, 144));
            } else {
                telemetry.addData("Goal ball velocity", shootermath.findLateralVelocity(follower.getPose(), 0, 144));
                currentShootVelocity = shootermath.ballVelocityToFlywheel(shootermath.findLateralVelocity(follower.getPose(), 0, 144));
            }
            telemetry.addData("Calculated Shooter Speed", currentShootVelocity);
        }
        else{
            telemetry.addData("Constant Shooter Speed", currentShootVelocity);
        }

        launch(gamepad2.rightBumperWasPressed());

        LLResult limelightResult = limelight.getLatestResult();

        if (gamepad1.a) {
            if (limelightResult != null && limelightResult.isValid()) {
                double error = -limelightResult.getTx();
                if (Math.abs(error) >= 1.0) {
                    double steeringAdjust;
                    if (error < 0) {
                        steeringAdjust = -0.1 * error + MINIMUM_ADJUSTMENT;
                    }
                    else {
                        steeringAdjust = -0.1 * error - MINIMUM_ADJUSTMENT;
                    }
                    mecanumDrive(0, 0, steeringAdjust);
                }
            }
        }

        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        if (limelightResult != null && limelightResult.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = limelightResult.getFiducialResults();
            if (!aprilTags.isEmpty()) {
                LLResultTypes.FiducialResult aprilTag = aprilTags.get(0);
                telemetry.addData("Tag id", aprilTag.getFiducialId());
            }
            else {
                telemetry.addData("Tag id", "No tag");
            }
            telemetry.addData("tx", limelightResult.getTx());
            telemetry.addData("ty", limelightResult.getTy());
            telemetry.addData("ta", limelightResult.getTa());
        }
        else {
            telemetry.addData("Limelight result", "Invalid");
        }
        if (alliance != Alliance.UNKNOWN) {
            if (alliance == Alliance.RED) {
                trackingAngle = PoseTrig.angleBetweenPoses(follower.getPose(), new Pose(144,144));
            }
            else if (alliance == Alliance.BLUE) {
                trackingAngle = PoseTrig.angleBetweenPoses(follower.getPose(), new Pose(0,144));
            }
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("angleToShooter", Math.toDegrees(trackingAngle));
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

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_MIN_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                feeder.setPower(FULL_SPEED);
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
