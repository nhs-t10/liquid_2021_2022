package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class DistanceTest extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    Rev2mDistanceSensor backDist2;
    Rev2mDistanceSensor backDist1;
    Rev2mDistanceSensor leftDist2;
    Rev2mDistanceSensor leftDist1;
    Rev2mDistanceSensor rightDist1;
    Rev2mDistanceSensor rightDist2;
    int step = 1;
    ElapsedTime timer;

    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) { }
    }



    public void init() {
        FeatureManager.setIsOpModeRunning(true);
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        hands = new ManipulationManager(
                new CRServo[] {},
                new String[] {},
                new Servo[] {},
                new String[] {},
                new DcMotor[] {fl, fr, br, bl, dw},
                new String[] {"fl", "fr", "br", "bl", "dw"}
        );
        driver = new MovementManager(fl, fr, br, bl);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        driver.setDirection();
        timer = new ElapsedTime();
        backDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "backDist2");
        backDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "backDist1");
        leftDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "leftDist2");
        leftDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "leftDist1");
        rightDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "rightDist2");
        rightDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "rightDist1");
    }
    public void loop() {
        switch (step) {
            case(1):
                telemetry.addData("back2 cm", "%.2f cm", backDist2.getDistance(CM)); //cm distance
                telemetry.addData("back1 cm", "%.2f cm", backDist1.getDistance(CM));
                telemetry.addData("left1 cm", "%.2f cm", leftDist1.getDistance(CM));
                telemetry.addData("left2 cm", "%.2f cm", leftDist2.getDistance(CM));
                telemetry.addData("right1 cm", "%.2f cm", rightDist1.getDistance(CM));
                telemetry.addData("right2 cm", "%.2f cm", rightDist2.getDistance(CM));



            case(2):
                telemetry.addLine("Autonomous Complete");
                telemetry.addData("time", timer.milliseconds());
                telemetry.addData("Step #", step);
                telemetry.update();
                step++;
        }

        telemetry.addData("back range", String.format("%.01f cm", backDist2.getDistance(DistanceUnit.CM)));
        telemetry.addData("back range", String.format("%.01f cm", backDist1.getDistance(DistanceUnit.CM)));
        telemetry.addData("left range", String.format("%.01f cm", leftDist2.getDistance(DistanceUnit.CM)));
        telemetry.addData("left range", String.format("%.01f cm", leftDist1.getDistance(DistanceUnit.CM)));
        telemetry.update();
    }

}
