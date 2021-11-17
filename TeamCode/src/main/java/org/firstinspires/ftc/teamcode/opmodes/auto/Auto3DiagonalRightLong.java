package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class Auto3DiagonalRightLong extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    /*float [] omniValues = new float [4];*/
    int step = 1;
    ElapsedTime timer;
    public void delayDrive(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
                hands.setMotorPower("dw",0.5);
        }
        hands.setMotorPower("dw",0);

    }

    public void init() {
        FeatureManager.setIsOpModeRunning(true);
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        driver = new MovementManager(fl, fr, br, bl);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        driver.runToPosition();
    }
    public void loop() {
        switch (step) {
            case(1):
                driver.setTargetPositions(13, 13, 13, 13);
                step++;
                break;
            case(2):
                telemetry.addLine("Reached Opposing Wall");
                telemetry.addData("Step #", step);
                telemetry.update();
                step++;
                break;
            case(3):
                driver.setTargetPositions(1,-1,-1,1);
                step++;
                break;
            case(4):
                telemetry.addLine("Right Turn Complete");
                telemetry.addData("Step #", step);
                telemetry.update();
                step++;
                break;
            case(5):
                driver.setTargetPositions(6, 6, 6, 6);
                step++;
                break;
            case(6):
                telemetry.addLine("Reached Duck Wheel");
                telemetry.addData("Step #", step);
                telemetry.update();
                step++;
                break;
            case(7):
                delayDrive(10000);
                step++;
                break;
            case(8):
                telemetry.addLine("Duck Deployed");
                telemetry.addData("Step #", step);
                telemetry.update();
                step++;
                break;
            case(9):
                driver.setTargetPositions(-13, -13, -13, -13);
                step++;
                break;
            case(10):
                telemetry.addLine("Arrived at Warehouse. Autonomous Complete.");
                telemetry.addData("Step #", step);
                telemetry.update();
                step++;
                break;
        }
    }

}
