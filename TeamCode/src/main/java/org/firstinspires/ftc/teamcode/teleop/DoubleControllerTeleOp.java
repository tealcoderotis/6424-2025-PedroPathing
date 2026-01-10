package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.PoseDimensionConverter;
import org.firstinspires.ftc.teamcode.Regression;

@TeleOp(name = "SingleControllerTeleOp")
//@Disabled
public class DoubleControllerTeleOp extends OpMode {
    final double STOP_SPEED = 0.0;

    final double LAUNCHER_IDLE_VELOCITY = 0;
    final double LAUNCHER_MAX_VELOCITY = 1950;
    final double LAUNCHER_MIN_VELOCITY = 1500;
    final double LAUNCHER_SPINUP_VELOCITY = 1200;
    final double LAUNCHER_UNJAM_VELOCITY = 300; //Backwards
    final double FEEDER_INTAKE_VELOCITY = 2300;
    final double FEEDER_UNJAM_VELOCITY = 700; // Backwards
    final double FEEDER_LAUNCH_VELOCITY = 2300;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx feeder = null;
    private Follower follower;
    boolean lockOn = false;
    boolean slowMode = false;
    final double SLOW_RATIO = 3; // 3 times slower for manual adjustment of angle
    private Alliance alliance = Alliance.UNKNOWN;
    double xGoal = 144;
    //Angles are in radians
    double angle = 0; // Follows convention that counterclockwise is positive, which means that a positive angle is counteracted with positive rotate.
    // No need for past angle due to .getAngularVelocity
    double rotate = 0;
    final double Pcoeff = 1;
    final double Dcoeff = 0.2;

    private boolean useLimelight = false;
    private Limelight3A limelight;

    private boolean fieldCentric = false;
    private IMU imu;

    boolean useColorSensor = false;
    private NormalizedColorSensor colorSensor;
    boolean prevDetection = false;
    int artifactCount = -1;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {

        //Output Initialization
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

        //Input Initialization
        follower = Constants.createFollower(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(Globals.IMU_ORIENTATION));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(5);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();


        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (Globals.alliance != Alliance.UNKNOWN) {
            alliance = Globals.alliance;
            //follower.setStartingPose(Globals.endingPose.copy()); //This seems to not work
            if (Globals.alliance == Alliance.RED) {
                follower.setStartingPose(new Pose(97.108, 59.579, Math.toRadians(0)));
            } else {
                follower.setStartingPose(new Pose(46.892, 59.798, Math.toRadians(180)));
            }
        }
        else {
            if (gamepad1.bWasPressed()) {
                //Red starting pose
                follower.setStartingPose(new Pose(97.108, 59.579, Math.toRadians(0)));
                alliance = Alliance.RED;
                xGoal = 144;
            } else if (gamepad1.xWasPressed()) {
                //Blue starting pose
                follower.setStartingPose(new Pose(46.892, 59.798, Math.toRadians(180)));
                alliance = Alliance.BLUE;
                xGoal = 0;
            }
        }
        if (gamepad1.dpadDownWasPressed()) {
            fieldCentric = true;
        } else if (gamepad1.dpadUpWasPressed()) {
            fieldCentric = false;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            useLimelight = true;
        } else if (gamepad1.dpadRightWasPressed()) {
            useLimelight = false;
        }
        if (gamepad1.aWasPressed()) {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
            useColorSensor = true;
            artifactCount = 0;
        } else if (gamepad1.yWasPressed()) {
            useColorSensor = false;
            artifactCount = -1;
        }
        telemetry.addData("alliance", alliance.toString());
        telemetry.addData("field centric movement", fieldCentric);
        telemetry.addData("color sensor used", useColorSensor);
        telemetry.addData("limelight used", useLimelight);
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
        rotate = 0;
        if (!lockOn) {
            if (!slowMode) {
                rotate = gamepad1.right_stick_x;
            } else {
                rotate = gamepad1.right_stick_x/SLOW_RATIO;
            }
        } else {
            angle = follower.getPose().getHeading() - Math.atan2(144-follower.getPose().getY(), xGoal-follower.getPose().getX());
            angle = ((angle + Math.PI) % (2 * Math.PI)) - Math.PI; //Makes angle between -pi and pi
            telemetry.addData("angle", angle);
            telemetry.addData("angleVelocity", follower.getAngularVelocity());
            rotate = Pcoeff * angle + Dcoeff * follower.getAngularVelocity();
            if (Math.abs(angle) < 1 && !(gamepad1.right_stick_x == 0)) {
                slowMode = true;
                lockOn = false;
            }
        }
        if (fieldCentric) {
            mecanumFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
        } else {
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
        }
        if (useColorSensor) {
            //Find number of artifacts in robot
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            if (colors.red != 0 && colors.green != 0 && colors.blue != 0) {
                if (!prevDetection) {
                    prevDetection = true;
                    artifactCount = artifactCount + 1;
                }
            } else {
                prevDetection = false;
            }
            telemetry.addData("Artifacts Controlled", artifactCount);
        }
        if (gamepad2.a || artifactCount == 0) {
            feeder.setDirection(DcMotor.Direction.REVERSE);
            feeder.setVelocity(FEEDER_INTAKE_VELOCITY);
            if (launcher.getVelocity() > LAUNCHER_IDLE_VELOCITY) {
                feeder.setDirection(DcMotor.Direction.REVERSE);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
            }
        }
        if (gamepad2.x || artifactCount == 3) {
            artifactCount = 0;
            telemetry.addData("Goal Ball Velocity", LAUNCHER_IDLE_VELOCITY);
            launcher.setDirection(DcMotor.Direction.FORWARD);
            launcher.setVelocity(LAUNCHER_UNJAM_VELOCITY);
            telemetry.addData("Shooter Speed", LAUNCHER_UNJAM_VELOCITY * -1);
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setVelocity(FEEDER_UNJAM_VELOCITY);
            lockOn = true;
        }

        if (gamepad2.b) {
            telemetry.addData("Goal Ball Velocity", LAUNCHER_IDLE_VELOCITY);
            launcher.setDirection(DcMotor.Direction.REVERSE);
            launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
            telemetry.addData("Shooter Speed", LAUNCHER_IDLE_VELOCITY);
            lockOn = false;
            slowMode = false;
        }
        if (gamepad2.y) {
            launcher.setDirection(DcMotor.Direction.REVERSE);
            launcher.setVelocity(LAUNCHER_SPINUP_VELOCITY);
        }
        if (gamepad2.dpad_up) {
            telemetry.addData("Goal Ball Velocity", "MAXIMUM");
            launcher.setDirection(DcMotor.Direction.REVERSE);
            launcher.setVelocity(LAUNCHER_MAX_VELOCITY);
            telemetry.addData("Shooter Speed", "MAXIMUM");
        }

        if (gamepad2.dpad_down) {
            telemetry.addData("Goal Ball Velocity", "MINIMUM");
            launcher.setDirection(DcMotor.Direction.REVERSE);
            launcher.setVelocity(LAUNCHER_MIN_VELOCITY);
            telemetry.addData("Shooter Speed", "MINIMUM");
        }
        else {
            feeder.setPower(STOP_SPEED);
        }
        if (gamepad1.dpad_left) {
            double dist = 0;
            if (!(alliance == Alliance.UNKNOWN)) {
                dist = Math.sqrt(Math.pow(xGoal-follower.getPose().getX(),2)+Math.pow(144-follower.getPose().getY(),2));
            } else {
                telemetry.addData("Goal Ball Velocity", "UNKNOWN ALLIANCE");
            }
            double flywheelVelocity = Regression.getVelocityForDistance(dist);
            if (flywheelVelocity == 0) {
                telemetry.addLine("Data not available for current distance!");
            }
            launcher.setDirection(DcMotor.Direction.REVERSE);
            launcher.setVelocity(flywheelVelocity);
            telemetry.addData("Shooter Speed", flywheelVelocity);
        }

        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("dist", Math.sqrt(Math.pow(xGoal-follower.getPose().getX(),2)+Math.pow(144-follower.getPose().getY(),2)));
        follower.update();
        if (useLimelight) { //Limelight used for odometry correction to ensure accurate localization
            LLResult result = limelight.getLatestResult();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            // First, tell Limelight which way your robot is facing
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            limelight.updateRobotOrientation(robotYaw);
            if (result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2(); //mt2 is MegaTag2
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    follower.setPose(PoseConverter.pose2DToPose(PoseDimensionConverter.pose3DToPose2D(botpose_mt2), InvertedFTCCoordinates.INSTANCE));
                    //This correction should work IF the position of the limelight is configured on the web limeligh access.
                }
            }
        }
    }
    void mecanumFieldCentric(double forward, double strafe, double rotate) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double fieldForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        double fieldStrafe = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        mecanumDrive(fieldForward, fieldStrafe, rotate);
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
