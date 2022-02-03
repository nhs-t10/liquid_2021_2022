package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.google.blocks.ftcrobotcontroller.hardware.HardwareType.BNO055IMU;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.auxilary.PaulMath;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.managers.imu.ImuManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class TestingNewIdeas extends OpMode {
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

    public void driveToDistanceFrontForward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (frontDist.getDistance(DistanceUnit.CM) >= cm) {
            //wait
        }
        driver.stopDrive();


    }

    public void driveToDistanceBackForward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (backDist.getDistance(DistanceUnit.CM) <= cm) {
            //wait
        }
        driver.stopDrive();
    }
    public void driveToDistanceFrontBackward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (frontDist.getDistance(DistanceUnit.CM) <= cm) {
            //wait
        }
        driver.stopDrive();
    }
    public void driveToDistanceBackBackward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (backDist.getDistance(DistanceUnit.CM) >= cm) {
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
        frontDist = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontDist");
        backDist = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backDist");
    }
    public void loop() {
        switch (step) {
            case(1):
                driveToDistanceBackForward(0.5f, 45);
                step++;

                break;
            case(2):

                step++;
                break;
            case(3):
                driveToDistanceBackBackward(0.5f, 1);
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
        telemetry.addData("raw ultrasonic", frontDist.rawUltrasonic()); //ultrasonic data
        telemetry.addData("raw optical", frontDist.rawOptical()); //optical data
        telemetry.addData("cm optical", "%.2f cm", frontDist.cmOptical()); //cm distance? todo learn more
        telemetry.addData("cm", "%.2f cm", frontDist.getDistance(DistanceUnit.CM)); //cm distance
        telemetry.update();
    }

}
