package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DualMotor {
    private DcMotorEx motor1;
    private DcMotorEx motor2;

    public DualMotor(DcMotorEx motor1, DcMotor.Direction motor1Direction, DcMotorEx motor2, DcMotor.Direction motor2Direction) {
        this.motor1 = motor1;
        motor1.setDirection(motor1Direction);
        this.motor2 = motor2;
        motor2.setDirection(motor2Direction);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setVelocity(double velocity) {
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }

    public void stop() {
        setVelocity(0);
    }

    public double getAverageVelocity() {
        return (motor1.getVelocity() + motor2.getVelocity()) / 2;
    }

    public DcMotorEx getMotor1() {
        return motor1;
    }

    public DcMotorEx getMotor2() {
        return motor2;
    }
}