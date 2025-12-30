package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseTrig;

public class TurretRotator {
    private final DcMotorEx turretRotate;
    private static final Pose RED_GOAL_POSE = new Pose(144, 144);
    private static final Pose BLUE_GOAL_POSE = new Pose(0, 144);
    private double currentAngle;
    private Telemetry telemetry;
    private Alliance alliance;
    public TurretRotator(HardwareMap hardwareMap, Alliance alliance) {
        turretRotate = (DcMotorEx)hardwareMap.get(Globals.TURRET_ROTATE_HARDWARE_MAP_NAME);
        turretRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        updateAngle();
        this.alliance = alliance;
    }

    public TurretRotator(HardwareMap hardwareMap, Alliance alliance, Telemetry telemetry) {
        this(hardwareMap, alliance);
        this.telemetry = telemetry;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    private void updateAngle() {
        this.currentAngle = ((turretRotate.getCurrentPosition() / Globals.TICKS_PER_TURRET_MOTOR_REVOLUTION) * Globals.DEGREES_PER_TURRET_MOTOR_REVOLUTION) % 360;
        if (currentAngle < 0) {
            currentAngle = 360 - currentAngle;
        }
    }

    private int angleToPosition(double angle) {
        double angleDifference = angle - getAngleDegrees();
        int ticks = (int) ((angleDifference / Globals.DEGREES_PER_TURRET_MOTOR_REVOLUTION) * Globals.TICKS_PER_TURRET_MOTOR_REVOLUTION);
        return turretRotate.getCurrentPosition() + ticks;
    }

    public double getAngleDegrees() {
        updateAngle();
        return this.currentAngle;
    }

    public void update(Pose botPose) {
        updateAngle();
        if (alliance != Alliance.UNKNOWN) {
            double targetAngle;
            if (alliance == Alliance.RED) {
                targetAngle = Math.toDegrees(PoseTrig.angleBetweenPoses(botPose, RED_GOAL_POSE));
            }
            else {
                targetAngle = Math.toDegrees(PoseTrig.angleBetweenPoses(botPose, BLUE_GOAL_POSE));
            }
            int ticks = angleToPosition(targetAngle);
            turretRotate.setTargetPosition(ticks);
            turretRotate.setPower(1);
            if (telemetry != null) {
                telemetry.addData("Angle to goal", targetAngle);
            }
        }
        turretRotate.setPower(0);
    }

    public void stop() {
        turretRotate.setPower(0);
    }
}
