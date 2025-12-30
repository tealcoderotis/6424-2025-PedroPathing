package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.ShooterMath;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "SingleControllerTeleOp")
//@Disabled
public class SingleControllerTeleOp extends OpMode {
    ShooterMath shootermath;
    final double STOP_SPEED = 0.0;

    final double LAUNCHER_IDLE_VELOCITY = 0;
    final double LAUNCHER_MAX_VELOCITY = 1950;
    final double LAUNCHER_MIN_VELOCITY = 1500;
    final double LAUNCHER_SPINUP_VELOCITY = 1200;
    final double FEEDER_INTAKE_VELOCITY = 1700;
    final double FEEDER_UNJAM_VELOCITY = 700;
    final double FEEDER_LAUNCH_VELOCITY = 1700;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx feeder = null;
    private Follower follower;
    boolean lockOn = false;
    double[] dataDistances = {50.996767, 69.574177, 84.84786, 102.93725, 118.13214};
    int [] dataShooterSpeeds = {1360, 1420, 1530, 1650, 1740};
    private Alliance alliance = Alliance.UNKNOWN;
    //Angles are in radians
    double angle = 0; // Follows convention that counterclockwise is positive, which means that a positive angle is counteracted with positive rotate.
    // No need for past angle due to .getAngularVelocity
    double rotate = 0;
    final double Pcoeff = 1;
    final double Dcoeff = 1;


    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        shootermath = new ShooterMath(telemetry);
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
        telemetry.addData("heading", follower.getHeading());
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(false);
    }

    @Override
    public void loop() {
        if (!lockOn) {
            rotate = gamepad1.right_stick_x;
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
        } else {
            angle = follower.getPose().getHeading() - Math.atan2(144-follower.getPose().getY(), 144-follower.getPose().getX());
            telemetry.addData("angle", angle);
            telemetry.addData("angleVelocity", follower.getAngularVelocity());
            rotate = Pcoeff * angle + Dcoeff * follower.getAngularVelocity();
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
        }

        if (gamepad1.a) {
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setVelocity(FEEDER_INTAKE_VELOCITY);
            if (launcher.getVelocity() > LAUNCHER_IDLE_VELOCITY) {
                feeder.setDirection(DcMotor.Direction.FORWARD);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
            }
        }
        if (gamepad1.x) {
            launcher.setDirection(DcMotor.Direction.REVERSE);
            telemetry.addData("Goal Ball Velocity", LAUNCHER_IDLE_VELOCITY);
            launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
            telemetry.addData("Shooter Speed", LAUNCHER_IDLE_VELOCITY);
            feeder.setDirection(DcMotor.Direction.REVERSE);
            feeder.setVelocity(FEEDER_UNJAM_VELOCITY);
            lockOn = true;
        }

        if (gamepad1.b) {
            telemetry.addData("Goal Ball Velocity", LAUNCHER_IDLE_VELOCITY);
            launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
            telemetry.addData("Shooter Speed", LAUNCHER_IDLE_VELOCITY);
            lockOn = false;
        }
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_SPINUP_VELOCITY);
        }
        if (gamepad1.dpad_up) {
            telemetry.addData("Goal Ball Velocity", "MAXIMUM");
            launcher.setVelocity(LAUNCHER_MAX_VELOCITY);
            telemetry.addData("Shooter Speed", "MAXIMUM");
        }

        if (gamepad1.dpad_down) {
            telemetry.addData("Goal Ball Velocity", "MINIMUM");
            launcher.setVelocity(LAUNCHER_MIN_VELOCITY);
            telemetry.addData("Shooter Speed", "MINIMUM");
        }
        else {
            feeder.setPower(STOP_SPEED);
        }
        if (gamepad1.dpad_left) {
            double dist = 0;
            if (alliance == Alliance.RED) {
                dist = Math.sqrt(Math.pow(144-follower.getPose().getX(),2)+Math.pow(144-follower.getPose().getY(),2));
            } else if (alliance == Alliance.BLUE) {
                dist = Math.sqrt(Math.pow(0-follower.getPose().getX(),2)+Math.pow(144-follower.getPose().getY(),2));
            } else {
                telemetry.addData("Goal Ball Velocity", "UNKNOWN ALLIANCE");
            }
            double flywheelVelocity = 0;
            for (int i = 1; i < dataDistances.length; i++) {
                if (dist < dataDistances[i] && dist >= dataDistances[i-1]) {
                    flywheelVelocity = dataShooterSpeeds[i] + (dataShooterSpeeds[i]-dataShooterSpeeds[i-1])/(dataDistances[i]-dataDistances[i-1]) * (dist - dataDistances[i]);
                }
            }
            if (flywheelVelocity == 0) {
                telemetry.addLine("Data not available for current distance!");
            }
            /*if (dist < 51) {
                flywheelVelocity = 1360 + 5.89098 * (dist - 50.99677);
            } else if (dist < 69.57418) {
                flywheelVelocity = 1420 + 3.22973 * (dist - 69.57418);
            } else if (dist < 84.84786) {
                flywheelVelocity = 1530 + 7.20193 * (dist - 84.84786);
            } else if (dist < 102.93725) {
                flywheelVelocity = 1650 + 6.63372 * (dist - 102.93725);
            } else if (dist < 118.13214) {
                flywheelVelocity = 1740 + 5.923044 * (dist - 118.13214);
            } else {
                flywheelVelocity = 1740 + 5.89098 * (dist - 118.13214);
            }*/
            launcher.setVelocity(flywheelVelocity);
            telemetry.addData("Shooter Speed", flywheelVelocity);
        }
        if (gamepad1.right_bumper){
            follower.setPose((new Pose(110.36335877862595, 134.10687022900763, 0)));
        }

        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("dist", Math.sqrt(Math.pow(144-follower.getPose().getX(),2)+Math.pow(144-follower.getPose().getY(),2)));
        follower.update();
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
}
