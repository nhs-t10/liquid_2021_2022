package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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
public class RangeSensorLeftAuto extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    ModernRoboticsI2cRangeSensor frontDist;
    ModernRoboticsI2cRangeSensor backDist;
    int step = 1;
    ElapsedTime timer;

    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) { }
    }

    public void driveToDist(ModernRoboticsI2cRangeSensor sensor, float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (sensor.getDistance(DistanceUnit.CM) >= cm) {
            //wait
        }
        driver.stopDrive();
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
        driver.runUsingEncoders();
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        driver.setDirection();
        timer = new ElapsedTime();
        frontDist = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_dist");
        backDist = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "back_dist");
    }
    public void loop() {
        switch (step) {
            case(1):
                driveToDist(backDist, 0.5f, 20);
                step++;
                break;
            case(2):
                hands.setMotorPower("dw", 0.5);
                delay(5000);
                hands.setMotorPower("dw",0);
                step++;
                break;
            case(3):
                driveToDist(frontDist, -0.5f, 20);
                step++;
                break;
            case(4):
                telemetry.addLine("Autonomous Complete");
                telemetry.addData("time", timer.milliseconds());
                telemetry.addData("Step #", step);
                telemetry.update();
        }
        telemetry.addLine("Encoder Values");
        telemetry.addData("fl pos", driver.flGetTicks());
        telemetry.addData("fr pos", driver.frGetTicks());
        telemetry.addData("bl pos", driver.blGetTicks());
        telemetry.addData("br pos", driver.brGetTicks());
        telemetry.addData("front raw ultrasonic", frontDist.rawUltrasonic()); //ultrasonic data
        telemetry.addData("front raw optical", frontDist.rawOptical()); //optical data
        telemetry.addData("front cm optical", "%.2f cm", frontDist.cmOptical()); //cm distance? todo learn more
        telemetry.addData("front cm", "%.2f cm", frontDist.getDistance(DistanceUnit.CM)); //cm distance
        telemetry.addData("back raw ultrasonic", backDist.rawUltrasonic()); //ultrasonic data
        telemetry.addData("back raw optical", backDist.rawOptical()); //optical data
        telemetry.addData("back cm optical", "%.2f cm", backDist.cmOptical()); //cm distance? todo learn more
        telemetry.addData("back cm", "%.2f cm", backDist.getDistance(DistanceUnit.CM)); //cm distance
        telemetry.update();
    }
}
