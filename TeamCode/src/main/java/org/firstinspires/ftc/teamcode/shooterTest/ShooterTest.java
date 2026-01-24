package org.firstinspires.ftc.teamcode.shooterTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean hasShooted = false;
        ShooterIntake shooterIntake = new ShooterIntake(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (!hasShooted) {
                shooterIntake.beginShooting(3);
                hasShooted = true;
            }
        }
    }
}
