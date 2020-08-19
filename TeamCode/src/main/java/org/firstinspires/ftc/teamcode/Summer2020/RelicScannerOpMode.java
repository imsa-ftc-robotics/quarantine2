package org.firstinspires.ftc.teamcode.Summer2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RelicScannerOpMode extends LinearOpMode {
    public void runOpMode() {
        while (true) {
            telemetry.addData("Result",  RelicScanner.scan().toString());
            telemetry.update();
        }
    }
}
