package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class TestBotTeleOp extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    private boolean rintakeRunning = false;
    private boolean lintakeRunning = false;
    private boolean rBumperDown = false;
    private boolean lBumperDown = false;
    boolean intakedown = true;
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
        Servo isl = hardwareMap.get(Servo.class, "isl");
        Servo isr = hardwareMap.get(Servo.class, "isr");
        DcMotor is = hardwareMap.get(DcMotor.class, "is");


        driver = new MovementManager(fl, fr, br, bl);

        hands = new ManipulationManager(
                new CRServo[] {},
                new String[] {},
                new Servo[] {isl, isr},
                new String[] {"isl", "isr"},
                new DcMotor[] {fl, fr, br, bl, is},
                new String[] {"fl", "fr", "br", "bl", "is"}
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

        input.registerInput("intake down",
                new ButtonNode("left_bumper")
        );
        input.registerInput("intake up",
                new ButtonNode("right_bumper")
        );
        input.registerInput("intake",
                new ButtonNode("right_trigger")
        );
        input.registerInput("outtake",
                new ButtonNode("left_trigger")
        );
        driver.setDirection();
    }

    @Override
    public void loop() {
        input.update();

        driver.downScale(0.5f);
        driver.testDriveOmni(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/2f);

        if (input.getBool("intake")) {
            hands.setMotorPower("is", -1);
        } else if (input.getBool("outtake")) {
            hands.setMotorPower("is", 1);
        } else {
            hands.setMotorPower("is", 0);
        }

        if (input.getBool("intake up") && intakedown) {
            hands.setServoPosition("isl", 2); // todo: update intake system for correct position
            hands.setServoPosition("isr", 2);
            intakedown = !intakedown;
        } else if (input.getBool("intake down") && !intakedown) {
            hands.setServoPosition("isl", 1);
            hands.setServoPosition("isr", 1);
            intakedown = !intakedown;
        }

        telemetry.addLine("Encoder Values");
        telemetry.addData("fl pos", driver.flGetTicks());
        telemetry.addData("fr pos", driver.frGetTicks());
        telemetry.addData("bl pos", driver.blGetTicks());
        telemetry.addData("br pos", driver.brGetTicks());
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
