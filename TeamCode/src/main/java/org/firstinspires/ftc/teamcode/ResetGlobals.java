package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Reset")
public class ResetGlobals extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Press A to reset globals");
            telemetry.addLine("Press B to reset turret encoder");
            if (gamepad1.aWasPressed()) {
                Globals.resetGlobals();
                telemetry.addLine("Globals reset");
            }
            if (gamepad1.bWasPressed()) {
                DcMotorEx turretRotate = (DcMotorEx)hardwareMap.get(Globals.TURRET_ROTATE_HARDWARE_MAP_NAME);
                turretRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("Turret encoder reset");
            }
            telemetry.update();
        }
        Globals.resetGlobals();
    }
}
