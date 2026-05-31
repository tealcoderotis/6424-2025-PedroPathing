package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "VariableShootSpeed")
public class VariableShootSpeed extends OpMode{
    double launcherVelocity = 1200;
    final double FEEDER_LAUNCH_VELOCITY = 3000;
    final double FEEDER_INTAKE_VELOCITY = 3000;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo stopper;
    private DcMotorEx launcher = null;
    private DcMotorEx feeder = null;
    ElapsedTime feederTimer = new ElapsedTime();
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        stopper = hardwareMap.get(Servo.class, "gateServo");
        ///hood = new Hood(hardwareMap.get(Servo.class, "hoodServo"));
        stopper.setPosition(1);
    }
    public void loop() {
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;
        mecanumDrive(-leftStickY, leftStickX, rightStickX);
        if (gamepad1.right_trigger >= 0.1) { // SHOOTER TO PROPER VELOCITY (SET WITH DPAD)
            launcher.setVelocity(launcherVelocity);
        }
        if (gamepad1.left_trigger >= 0.1) { //INTAKE
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setVelocity(FEEDER_INTAKE_VELOCITY);
        } else if (!gamepad1.right_bumper) {
            feeder.setVelocity(0);
        }
        if (gamepad1.right_bumper) { //GATE HOLD OPEN
            stopper.setPosition(1);
            feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
        }
        if (gamepad1.dpadUpWasPressed()) {
            launcherVelocity += 10;
        }
        if (gamepad1.dpadDownWasPressed()) {
            launcherVelocity -= 10;
        }
        telemetry.addData("velocity", launcherVelocity);
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
