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
    ModernRoboticsI2cRangeSensor rangeSensor;
    int step = 1;
    ElapsedTime timer;

    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) { }
    }

    public void driveToDistanceForward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (rangeSensor.getDistance(DistanceUnit.CM) >= cm) {
            //wait
        }
        driver.stopDrive();
    }

    public void driveToDistanceBackward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (rangeSensor.getDistance(DistanceUnit.CM) <= cm) {
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
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }
    public void loop() {
        switch (step) {
            case(1):
                driveToDistanceForward(0.5f, 20);
                step++;
                break;
            case(2):
                hands.setMotorPower("dw", 0.5);
                delay(5000);
                hands.setMotorPower("dw",0);
                step++;
                break;
            case(3):
                driveToDistanceBackward(-0.5f, 20);
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
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic()); //ultrasonic data
        telemetry.addData("raw optical", rangeSensor.rawOptical()); //optical data
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical()); //cm distance? todo learn more
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM)); //cm distance
        telemetry.update();
    }

}
