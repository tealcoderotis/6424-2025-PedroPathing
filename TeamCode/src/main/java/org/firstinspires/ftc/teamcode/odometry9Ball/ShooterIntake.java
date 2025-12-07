package org.firstinspires.ftc.teamcode.odometry9Ball;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.VoltagePowerCompensator;

public class ShooterIntake {
    //TODO Figure out how to shoot continuously
    private DcMotor indexer;
    private DcMotorEx shooter;
    private Timer shootTimer;
    private Timer intakeTimer;
    private boolean isShooterBusy = false;
    private boolean isReving = false;
    private boolean isIntaking = false;
    private boolean isIntakeContinuous = false;
    private boolean isIntakeMovingBack = false;
    private static final int SHOOTING_TIME = 1500;
    private static final int INDEX_TIME = 250;
    private static final int INTAKE_TIME = 250;
    private static final int REV_TIME = 1500;
    private static final int INTAKE_END_TIME = 300;
    private static final double SHOOTER_SPEED = 1400;
    private static final double INDEXER_POWER = 0.65;
    private static final double INDEXER_BACK_POWER = -0.35;
    private static final double SHOOTER_BACK_POWER = 0.5;
    private int ballsToShoot = 0;
    private int currentBall = -1;
    private double shooterSpeed = 0;
    private VoltagePowerCompensator voltageCompensator;
    private Telemetry telemetry;
    private boolean hasIndexed = false;
    public ShooterIntake(HardwareMap hardwareMap) {
        shootTimer = new Timer();
        intakeTimer = new Timer();
        indexer = (DcMotor)hardwareMap.get("feeder");
        shooter = (DcMotorEx)hardwareMap.get("launcher");
        voltageCompensator = new VoltagePowerCompensator(hardwareMap);
        resetEncoders();
    }

    private void resetEncoders() {
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

    public ShooterIntake(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    //only called once when we start shooting
    public void beginShooting(int ballsToShoot, double shooterSpeed) {
        isIntaking = false;
        this.ballsToShoot = ballsToShoot;
        currentBall = 0;
        if (!isReving) {
            shootTimer.resetTimer();
            shooter.setVelocity(-shooterSpeed);;
            isReving = true;
        }
        hasIndexed = false;
        isShooterBusy = true;
    }

    public void beginShooting(int ballsToShoot) {
        beginShooting(ballsToShoot, SHOOTER_SPEED);
    }

    public void beginReving(double shooterSpeed) {
        if (!isIntakeMovingBack) {
            currentBall = -1;
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
        indexer.setPower(voltageCompensator.compensate(INDEXER_POWER));
        shooter.setPower(voltageCompensator.compensate(SHOOTER_BACK_POWER));
        isShooterBusy = true;
    }

    //called every loop; checks the elapsed time and moves on to the next ball or stops shooting accordingly
    public void update() {
        if (isShooterBusy) {
            if (isIntakeMovingBack) {
                if (intakeTimer.getElapsedTime() >= INTAKE_END_TIME) {
                    indexer.setPower(0);
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
                    if (shootTimer.getElapsedTime() >= REV_TIME && currentBall != -1) {
                        isReving = false;
                        indexer.setPower(voltageCompensator.compensate(INDEXER_POWER));
                        shootTimer.resetTimer();
                    }
                }
                else {
                    if (shootTimer.getElapsedTime() >= INDEX_TIME && !hasIndexed) {
                        indexer.setPower(0);
                        currentBall ++;
                        if (currentBall >= ballsToShoot) {
                            stop();
                        }
                        hasIndexed = true;
                    }
                    if (shootTimer.getElapsedTime() >= SHOOTING_TIME) {
                        indexer.setPower(voltageCompensator.compensate(INDEXER_POWER));
                        shootTimer.resetTimer();
                        hasIndexed = false;
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
        shooter.setPower(0);
        indexer.setPower(0);
        isShooterBusy = false;
        isReving = false;
        isIntaking = false;
        hasIndexed = false;
        currentBall = -1;
        shooterSpeed = 0;
    }

    public void stopIntaking() {
        indexer.setPower(voltageCompensator.compensate(INDEXER_BACK_POWER));
        shooter.setPower(voltageCompensator.compensate(SHOOTER_BACK_POWER));
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