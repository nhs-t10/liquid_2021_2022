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

@TeleOp
public class CorbinTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    private boolean precision = false;
    int dwSpeed = 0;
    ElapsedTime timer;

    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            // wait... wait... wait...
        }
    }

    public void autoMove() {
        // Lower servo to correct height
        hands.setServoPosition("it", /*todo get correct position*/ 1);
        // Spin wheel
        hands.setMotorPower("is", 1);
        // Sleep long enough to get item todo figure out correct length
        this.delay(1000);
        // Stop wheel
        hands.setMotorPower("is", 0);
        // Raise servo back up
        hands.setServoPosition("it", /*todo get correct position*/ 1);
    }

    @Override
    public void init() {
        /* Phone is labelled as Not Ready For Use */
        FeatureManager.setIsOpModeRunning(true);
        float [] omniValues = new float [4];
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        DcMotor is = hardwareMap.get(DcMotor.class, "is");
        Servo it = hardwareMap.get(Servo.class, "it");

        driver = new MovementManager(fl, fr, br, bl);

        hands = new ManipulationManager(new CRServo[] {}, new String[] {}, new Servo[] {}, new String[] {}, new DcMotor[] {fl, fr, br, bl, dw}, new String[] {"fl", "fr", "br", "bl", "dw"});

        input = new InputManager(gamepad1, gamepad2);

        input.registerInput("drivingControls",
                new MultiInputNode(
                        new JoystickNode("left_stick_y"),
                        new JoystickNode("left_stick_x"),
                        new JoystickNode("right_stick_x"),
                        new JoystickNode("right_stick_y")
                )
        );


        input.registerInput("TestDrive",
                new ButtonNode("a")
        );
        input.registerInput("duckWheelRight",
                new ButtonNode("b")
        );

        input.registerInput("duckWheelLeft",
                new ButtonNode("x")
        );

        input.registerInput("taunts",
                new MultiInputNode(
                        new ButtonNode("dpad_up"),
                        new ButtonNode("dpad_left"),
                        new ButtonNode("dpad_right"),
                        new ButtonNode("dpad_down")
                )
        );

        input.registerInput("autoMove",
                new ButtonNode("leftbumper")
        );
    }

    @Override
    public void loop() {
        input.update();

        driver.driveOmni(input.getFloatArrayOfInput("drivingControls"));


        if (input.getBool("duckWheelRight")) {
            hands.setMotorPower("dw", -0.5);
            if (gamepad1.left_bumper) {
                hands.setMotorPower("dw", -1);
            }
        } else if (input.getBool("duckWheelLeft")) {
            hands.setMotorPower("dw", 0.5);
            if (gamepad1.left_bumper) {
                hands.setMotorPower("dw", 1);
            }
        } else {
            hands.setMotorPower("dw", 0);
        }
        if (input.getBool("TestDrive")) {
            driver.driveRaw(0.5f, 0.5f, 0.5f, 0.5f);
        }
        if (input.getBool("autoMove")) {
            this.autoMove();
        }


        //driver.driveOmni(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);


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
