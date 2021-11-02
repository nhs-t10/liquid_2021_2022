package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.input.InputManager;
import org.firstinspires.ftc.teamcode.managers.input.nodes.ButtonNode;
import org.firstinspires.ftc.teamcode.managers.input.nodes.InputManagerInputNode;
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
public class ExampleTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    private boolean precision = false;
    int driveSpeed = 0;

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
        //DcMotor re = hardwareMap.get(DcMotor.class, "re");

        driver = new MovementManager(fl, fr, br, bl);


        hands = new ManipulationManager(new CRServo[] {}, new String[] {}, new Servo[] {}, new String[] {}, new DcMotor[] {fl, fr, br, bl}, new String[] {"fl", "fr", "br", "bl"});

        input = new InputManager(gamepad1, gamepad2);

        input.registerInput("drivingControls",
                new MultiInputNode(
                        new JoystickNode("left_stick_y"),
                        new JoystickNode("left_stick_x"),
                        new JoystickNode("right_stick_x")
                )
        );

        input.registerInput("TestDrive",
                new ButtonNode("a")
        );

        input.registerInput("turnLeft",
                new ButtonNode("x")
        );
        input.registerInput("turnRight",
                new ButtonNode("b")
        );

        input.registerInput("taunts",
                new MultiInputNode(
                        new ButtonNode("dpad_up"),
                        new ButtonNode("dpad_left"),
                        new ButtonNode("dpad_right"),
                        new ButtonNode("dpad_down")
                )
        );
    }

    @Override
    public void loop() {
        input.update();
        driver.driveOmni(input.getFloatArrayOfInput("drivingControls"));
        if (input.getBool("TestDrive")) {
            driver.driveRaw(0.5f, 0.5f,0.5f, 0.5f);
        }
        if (input.getBool("turnLeft")) {
            driver.driveRaw(-0.5f, 0.5f, 0.5f, -0.5f);
        }
        if (input.getBool("turnRight")) {
            driver.driveRaw(0.5f, -0.5f, -0.5f, 0.5f);
        }

        /*
        if (input.getFloat("left_stick_x") < 0f && input.getFloat("left_stick_y") < 0f) {
            driver.driveRaw(-0.5f, -0.5f, 0.5f);
        }
        else if (input.getFloat("left_stick_x") < 0f && input.getFloat("left_stick_y") > 0f) {
            driver.driveOmni(0.5f, -0.5f, 0f);
        }
        else if (input.getFloat("left_stick_x") > 0f && input.getFloat("left_stick_y") > 0f) {
            driver.driveOmni(0.5f, 0.5f, 0f);
        }
        else if (input.getFloat("left_stick_x") > 0f && input.getFloat("left_stick_y") < 0f) {
            driver.driveOmni(-0.5f, 0.5f, 0f);
        }

         */
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
