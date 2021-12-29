package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.input.InputManager;
import org.firstinspires.ftc.teamcode.managers.input.nodes.ButtonNode;
import org.firstinspires.ftc.teamcode.managers.input.nodes.JoystickNode;
import org.firstinspires.ftc.teamcode.managers.input.nodes.MultiInputNode;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyGamepad;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyHardwareMap;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyTelemetry;
import org.junit.Test;

/*
 * A: Turn 180
 * LB: Toggle Servo
 * RB: Toggle Intake Motors
 * L2: Duck Wheel Left
 * R2: Duck Wheel Right
 * LStick: Omni
 * RStick: X: turn
 */

@TeleOp
public class EncoderTesterTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    ModernRoboticsI2cRangeSensor rangeSensor;
    private InputManager input;


    ElapsedTime timer;
    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            //do nothing
        }


    }

    @Override
    public void init() {
        /* Phone is labelled as Not Ready For Use */
        FeatureManager.setIsOpModeRunning(true);
        timer = new ElapsedTime();
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        DcMotor is = hardwareMap.get(DcMotor.class, "is");
        Servo it = hardwareMap.get(Servo.class, "it");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");

        driver = new MovementManager(fl, fr, br, bl);

        hands = new ManipulationManager(
                new CRServo[] {},
                new String[] {},
                new Servo[] {it},
                new String[] {"it"},
                new DcMotor[] {fl, fr, br, bl, dw, is},
                new String[] {"fl", "fr", "br", "bl", "dw", "is"}
        );

        input = new InputManager(gamepad1, gamepad2);

        input.registerInput("drivingControls",
                new MultiInputNode(
                        new JoystickNode("left_stick_y"),
                        new JoystickNode("left_stick_x"),
                        new JoystickNode("right_stick_x"),
                        new JoystickNode("right_stick_y")
                )
        );
        driver.setDirection();



    }

    @Override
    public void loop() {
        input.update();

        driver.downScale(0.5f);
        driver.testDriveOmni(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/2f);

        // Toggle input motors

        if (gamepad1.y) {
            hands.setServoPosition("it", 0.2);
        }
        if (gamepad1.b) {
            hands.setServoPosition("it", 0.216);
            delay(100);
            hands.setServoPosition("it", 0.429);
            delay(100);
            hands.setServoPosition("it", 0.65);
        }
        if (gamepad1.x) {
            hands.setServoPosition("it", 0.4);
        }

        if (gamepad1.a) {

        }



        /*
        if (gamepad1.x) {
            driver.driveRaw(0.5f,0,0,0);
        }
        if (gamepad1.y) {
            driver.driveRaw(0f,0.5f,0,0);
        }

         */

        telemetry.addLine("Encoder Values");
        telemetry.addData("fl pos", driver.flGetTicks());
        telemetry.addData("fr pos", driver.frGetTicks());
        telemetry.addData("bl pos", driver.blGetTicks());
        telemetry.addData("br pos", driver.brGetTicks());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM)); //cm distance
        telemetry.update();
    }

    public void stop() {
        FeatureManager.setIsOpModeRunning(false);
    }

    @Test
    public void test() {
        this.hardwareMap = new DummyHardwareMap();
        this.gamepad1 = new DummyGamepad();
        this.gamepad2 = new DummyGamepad();
        this.telemetry = new DummyTelemetry();

        long startTime = System.currentTimeMillis();

        this.start();
        this.init();
        this.init_loop();
        for(int i = 0; i < 3; i++) {
            this.time = (System.currentTimeMillis() - startTime) * 0.001;
            this.loop();
        }
        this.stop();
    }
}
