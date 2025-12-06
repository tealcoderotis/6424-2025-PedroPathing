package org.firstinspires.ftc.teamcode.shooterTest;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.VoltagePowerCompensator;


//handle our intake and shooter
public class ShooterIntake {
    private DcMotor indexer;
    private DcMotorEx shooter;
    private Timer shootTimer;
    private boolean isShooterBusy = false;
    private boolean isReving = false;
    private boolean isIntaking = false;
    private boolean isIntakeContinuous = false;
    private boolean isIntakeMovingBack = false;
    private static final int SHOOTING_TIME = 2000;
    private static final int INDEX_TIME = 300;
    private static final int INTAKE_TIME = 500;
    private static final int REV_TIME = 2000;
    private static final int INTAKE_END_TIME = 250;
    private static final double SHOOTER_SPEED = -1510;
    private static final double INDEXER_POWER = 0.65;
    private static final double INDEXER_BACK_POWER = -0.65;
    private int ballsToShoot = 0;
    private int currentBall = -1;
    private VoltagePowerCompensator voltageCompensator;
    private Telemetry telemetry;
    public ShooterIntake(HardwareMap hardwareMap) {
        shootTimer = new Timer();
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
    public void beginShooting(int ballsToShoot) {
        isIntaking = false;
        this.ballsToShoot = ballsToShoot;
        currentBall = 0;
        if (!isReving) {
            shootTimer.resetTimer();
            shooter.setVelocity(SHOOTER_SPEED);
            isReving = true;
        }
        isShooterBusy = true;
    }

    public void beginReving() {
        currentBall = -1;
        isIntaking = false;
        shootTimer.resetTimer();
        shooter.setVelocity(SHOOTER_SPEED);
        isShooterBusy = true;
        isReving = true;
    }

    //only called once when we start intaking
    public void beginIntaking(boolean continuous) {
        isIntakeMovingBack = false;
        isIntakeContinuous = continuous;
        isIntaking = true;
        shootTimer.resetTimer();
        indexer.setPower(voltageCompensator.compensate(INDEXER_POWER));
        isShooterBusy = true;
    }

    //called every loop; checks the elapsed time and moves on to the next ball or stops shooting accordingly
    public void update() {
        if (isShooterBusy) {
            if (isIntaking) {
                if (!isIntakeContinuous) {
                    if (shootTimer.getElapsedTime() >= INTAKE_TIME) {
                        indexer.setPower(0);
                        isShooterBusy = false;
                    }
                }
                if (isIntakeMovingBack) {
                    if (shootTimer.getElapsedTime() >= INTAKE_END_TIME) {
                        stop();
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
                    if (shootTimer.getElapsedTime() >= INDEX_TIME) {
                        indexer.setPower(0);
                    }
                    if (shootTimer.getElapsedTime() >= SHOOTING_TIME) {
                        currentBall ++;
                        if (currentBall < ballsToShoot) {
                            indexer.setPower(voltageCompensator.compensate(INDEXER_POWER));
                            shootTimer.resetTimer();
                        }
                        else {
                            stop();
                        }
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
        currentBall = -1;
    }

    public void stopIntaking() {
        indexer.setPower(voltageCompensator.compensate(INDEXER_BACK_POWER));
        shootTimer.resetTimer();
        isIntakeMovingBack = true;
    }

    //called every loop while the shooter/intake is active; returns true if it is currently shooting or intaking
    public boolean isBusy() {
        return isShooterBusy;
    }
}
