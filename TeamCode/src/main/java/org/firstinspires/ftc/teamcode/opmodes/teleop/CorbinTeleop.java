package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
public class CorbinTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    private boolean intakePosition;
    private boolean intakeRunning;

    @Override
    public void init() {
        /* Phone is labelled as Not Ready For Use */
        FeatureManager.setIsOpModeRunning(true);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        DcMotor is = hardwareMap.get(DcMotor.class, "is");
        Servo it = hardwareMap.get(Servo.class, "it");

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

        input.registerInput("toggleTray",
                new ButtonNode("left_bumper")
        );
        input.registerInput("toggleIn",
                new ButtonNode("right_bumper")
        );
        input.registerInput("duckWheelRight",
                new ButtonNode("right_trigger")
        );
        input.registerInput("duckWheelLeft",
                new ButtonNode("left_trigger")
        );
        input.registerInput("spin",
                new ButtonNode("a")
        );
    }

    @Override
    public void loop() {
        input.update();

        driver.driveOmni(input.getFloatArrayOfInput("drivingControls"));

        if (input.getBool("duckWheelRight")) {
            hands.setMotorPower("dw", -(input.getFloat("duckWheelRight")));
        } else if (input.getBool("duckWheelLeft")) {
            hands.setMotorPower("dw", (input.getFloat("duckWheelLeft")));
        } else {
            hands.setMotorPower("dw", 0);
        }
        // Spin 180
        if (input.getBool("spin")) {
            driver.driveRaw(1.0f, -1.0f, 1.0f, -1.0f);
            // todo spin robot correct amount
        }
        // Toggle input motors
        if (input.getBool("toggleIn")) {
            intakeRunning = !intakeRunning;
        }
        if (intakeRunning) {
            hands.setMotorPower("is", 1);
        } else {
            hands.setMotorPower("is", 0);
        }
        // Toggle input tray todo correct position numbers
        if (gamepad1.left_bumper) {
            intakePosition = !intakePosition;
        }
        if (intakePosition) {
            hands.setServoPosition("it", 1.0);
        } else {
            hands.setServoPosition("it", 0.5);
        }

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
