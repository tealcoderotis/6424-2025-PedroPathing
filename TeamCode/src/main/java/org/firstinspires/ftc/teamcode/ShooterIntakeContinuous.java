package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals;

public class ShooterIntakeContinuous {
    private final DcMotorEx indexer;
    private final DcMotorEx shooter;
    private final Servo gate;
    private final Timer shootTimer;
    private final Timer intakeTimer;
    private boolean isShooterBusy = false;
    private boolean isReving = false;
    private boolean isIntaking = false;
    private boolean isIntakeContinuous = false;
    private boolean isIntakeMovingBack = false;
    private static final int INDEX_TIME = 300;
    private static final int INTAKE_TIME = 250;
    private static final int INTAKE_END_TIME = Globals.INTAKE_BACK_TIME;
    private static final double SHOOTER_SPEED = Globals.SHOOTER_VELOCITY;
    private int shootingTime = -1;
    private double shooterSpeed = 0;
    private Telemetry telemetry;
    private final double GATE_OPEN_POSITION = 1;
    private final double GATE_CLOSE_POSITION = 0.5;
    public ShooterIntakeContinuous(HardwareMap hardwareMap) {
        shootTimer = new Timer();
        intakeTimer = new Timer();
        indexer = (DcMotorEx)hardwareMap.get("feeder");
        shooter = (DcMotorEx)hardwareMap.get("launcher");
        gate = (Servo)hardwareMap.get("gateServo");
        resetEncoders();
    }

    private void resetEncoders() {
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Globals.SHOOTER_PIDF);
        gate.setPosition(GATE_CLOSE_POSITION);
    }

    public void start() {
        indexer.setPower(Globals.FEEDER_IDLE_VELOCITY);
    }

    public ShooterIntakeContinuous(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    //only called once when we start shooting
    public void beginShooting(int ballsToShoot, double shooterSpeed) {
        isIntaking = false;
        shootingTime = INDEX_TIME * ballsToShoot;
        if (!isReving) {
            shootTimer.resetTimer();
            shooter.setVelocity(-shooterSpeed);
            isReving = true;
        }
        isShooterBusy = true;
    }

    public void beginShooting(int ballsToShoot) {
        beginShooting(ballsToShoot, SHOOTER_SPEED);
    }

    public void beginReving(double shooterSpeed) {
        if (!isIntakeMovingBack) {
            shootingTime = -1;
            isIntaking = false;
            shootTimer.resetTimer();
            shooter.setVelocity(-shooterSpeed);
            isShooterBusy = true;
        }
        this.shooterSpeed = shooterSpeed;
        isReving = true;
    }

    public void beginReving() {
        beginReving(SHOOTER_SPEED);
    }

    //only called once when we start intaking
    public void beginIntaking(boolean continuous) {
        isIntakeMovingBack = false;
        isIntakeContinuous = continuous;
        isIntaking = true;
        shootTimer.resetTimer();
        indexer.setPower(Globals.FEEDER_INTAKE_VELOCITY);
        shooter.setPower(Globals.SHOOTER_BACK_VELOCITY);
        isShooterBusy = true;
    }

    //called every loop; checks the elapsed time and moves on to the next ball or stops shooting accordingly
    public void update() {
        if (isShooterBusy) {
            if (isIntakeMovingBack) {
                if (intakeTimer.getElapsedTime() >= INTAKE_END_TIME) {
                    indexer.setPower(Globals.FEEDER_IDLE_VELOCITY);
                    shooter.setPower(0);
                    isIntakeMovingBack = false;
                    if (!isReving) {
                        stop();
                    }
                    else {
                        beginReving(this.shooterSpeed);
                    }
                }
            }
            if (isIntaking) {
                if (!isIntakeContinuous) {
                    if (shootTimer.getElapsedTime() >= INTAKE_TIME) {
                        indexer.setPower(0);
                        isShooterBusy = false;
                    }
                }
            }
            else {
                if (isReving) {
                    double differenceFromTarget = Math.abs(-shooter.getVelocity() - this.shooterSpeed);
                    if ((differenceFromTarget <= Globals.VELOCITY_TOLERANCE || shootTimer.getElapsedTime() >= Globals.REV_TIME) && shootingTime != -1) {
                        gate.setPosition(GATE_OPEN_POSITION);
                        isReving = false;
                        indexer.setPower(Globals.FEEDER_LAUNCH_VELOCITY);
                        shootTimer.resetTimer();
                    }
                }
                else {
                    if (shootTimer.getElapsedTime() >= shootingTime) {
                        stop();
                    }
                }
            }
        }
        if (telemetry != null) {
            telemetry.addData("Indexer Encoder Position", indexer.getCurrentPosition());
            telemetry.addData("Shooter Encoder Position", shooter.getCurrentPosition());
        }
    }

    //stops the shooter
    public void stop() {
        gate.setPosition(GATE_CLOSE_POSITION);
        shooter.setPower(0);
        indexer.setPower(Globals.FEEDER_IDLE_VELOCITY);
        isShooterBusy = false;
        isReving = false;
        isIntaking = false;
        shootingTime = -1;
        shooterSpeed = 0;
    }

    public void stopIntaking() {
        indexer.setPower(Globals.FEEDER_BACK_VELOCITY);
        shooter.setPower(Globals.SHOOTER_BACK_VELOCITY);
        intakeTimer.resetTimer();
        shootTimer.resetTimer();
        isIntaking = false;
        isIntakeMovingBack = true;
    }

    //called every loop while the shooter/intake is active; returns true if it is currently shooting or intaking
    public boolean isBusy() {
        return isShooterBusy;
    }
}