package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.input.InputManager;
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
public class RohanTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    Rev2mDistanceSensor backDist2;
    Rev2mDistanceSensor backDist1;
    Rev2mDistanceSensor leftDist1;
    Rev2mDistanceSensor leftDist2;
    int step = 2;
    int delayStep = -1;
    boolean runOnce = false;
    public ElapsedTime timer = new ElapsedTime();
    double endTime = timer.milliseconds();
    private boolean rintakeRunning = false;
    private boolean lintakeRunning = false;
    private boolean rBumperDown = false;
    private boolean lBumperDown = false;

    public void delayStopDrive(double delay) {
        while (timer.milliseconds() <= endTime) {

        }
        driver.stopDrive();
    }
    public void delayIntakeStop(double delay) {
        while (timer.milliseconds() <= endTime) {

        }
        driver.stopDrive();
    }
    public void delayStepDriveStop(double delay) {
        if (delayStep != step) {
            delayStep = step;
            endTime = timer.milliseconds() + delay;
        }
        if (timer.milliseconds() >= endTime) {
            driver.stopDrive();
            step++;
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
        CRServo isl = hardwareMap.get(CRServo.class, "isl");
        CRServo isr = hardwareMap.get(CRServo.class, "isr");
        Servo ill = hardwareMap.get(Servo.class, "ill");
        Servo ilr = hardwareMap.get(Servo.class, "ilr");
        backDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "backDist1");
        backDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "backDist2");
        leftDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "leftDist1");
        leftDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "leftDist2");


        driver = new MovementManager(fl, fr, br, bl);

        hands = new ManipulationManager(
            new CRServo[] {isl, isr},
            new String[] {"isl", "isr"},
            new Servo[] {ill, ilr},
            new String[] {"ill", "ilr"},
            new DcMotor[] {fl, fr, br, bl, dw},
            new String[] {"fl", "fr", "br", "bl", "dw"}
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
        switch(step) {
            case(1):
                driver.testDriveOmni(-0.25,0.25,0);
                if (leftDist1.getDistance(CM) <= 6 || leftDist2.getDistance(CM) <= 6) {
                    driver.driveRaw(0.5f, 0.5f, 0.5f, 0.5f);
                }
                delayStepDriveStop(1500);
                if (gamepad1.left_stick_button) {
                    step++;
                }
                break;
            case(2):
                input.update();
                telemetry.addLine("l Stick Values");
                telemetry.addData("lStickX", gamepad1.left_stick_x);
                telemetry.addData("lStickY", gamepad1.left_stick_y);
                driver.testDriveOmni(gamepad1.left_stick_y / 1.5, -gamepad1.left_stick_x / 1.5, -gamepad1.right_stick_x / 2.0);

                if (gamepad1.right_trigger > 0f) {
                    hands.setMotorPower("dw", -1);
                } else if (gamepad1.right_trigger == 0f) {
                    hands.setMotorPower("dw", 0);
                }
                if (gamepad1.left_trigger > 0f) {
                    hands.setMotorPower("dw", 1);
                } else if (gamepad1.left_trigger == 0f) {
                    hands.setMotorPower("dw", 0);
                }

                if (gamepad1.left_bumper) {
                    hands.setServoPower("isl", 1);
                    hands.setServoPower("isl", 0);
                    hands.setServoPower("isr", -1);
                    hands.setServoPower("isr", 0);
                } else if (gamepad1.right_bumper) {
                    hands.setServoPower("isl", -1);
                    hands.setServoPower("isr", 1);
                } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    hands.setServoPower("isl", 0);
                    hands.setServoPower("isr", 0);
                }
                if (gamepad1.b) {
                    hands.setServoPosition("ill", 0.95);
                    hands.setServoPosition("ilr", 0.05);
                }
                if (gamepad1.x) {
                    hands.setServoPosition("ill", 0.65);
                    hands.setServoPosition("ilr", 0.35);
                }
                if (gamepad1.a) {
                    hands.setServoPosition("ill", 0.85);
                    hands.setServoPosition("ilr", 0.15);
                }

                if (gamepad1.y) {
                    driver.testDriveOmni(0,-0.5,0);
                    delayStopDrive(1000);
                    hands.setServoPower("isl", 1);
                    hands.setServoPower("isl", 0);
                    hands.setServoPower("isr", -1);
                    hands.setServoPower("isr", 0);
                    delayIntakeStop(1000);
                }
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
