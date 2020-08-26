package org.firstinspires.ftc.teamcode.Summer2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RelicScannerOpMode extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        while (true) {
            telemetry.addData("Result",  RelicScanner.scan(this).toString());
            telemetry.update();
        }
    }
}
