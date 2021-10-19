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
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);



        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor re = hardwareMap.get(DcMotor.class, "re");

        driver = new MovementManager(fl, fr, br, bl);


        hands = new ManipulationManager(new CRServo[] {}, new String[] {}, new Servo[] {}, new String[] {}, new DcMotor[] {re}, new String[] {"re"});

        input = new InputManager(gamepad1, gamepad2);

        input.registerInput("drivingControls",
                new MultiInputNode(
                        new JoystickNode("left_stick_y"),
                        new JoystickNode("left_stick_x"),
                        new JoystickNode("right_stick_x")
                )
        );
        input.registerInput("PrecisionDriving",
                new ButtonNode("b")
        );
        input.registerInput("TestDrive",
                new ButtonNode("y")
        );
        input.registerInput("tdIncrease",
                new ButtonNode("a")
        );
        input.registerInput("tdDecrease",
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
    }

    @Override
    public void loop() {
        input.update();
        driver.driveOmni(input.getFloatArrayOfInput("drivingControls"));
        if (input.getBool("PrecisionDriving") == true && precision == false){
            driver.downScale(0.5f);
            precision = true;
        }
        else if (input.getBool("PrecisionDriving") == true && precision == true){
            precision = true;
        }
        else if (input.getBool("PrecisionDriving") == false && precision == true){
            driver.upScale(0.5f);
            precision = false;
        }
        else {
            precision = false;
        }
        if (input.getBool("tdIncrease")) {
            driveSpeed += 0.1;
        }
        else if (input.getBool("tdDecrease")) {
            driveSpeed -= 0.1;

        }
        else if (driveSpeed < 0) {
            driveSpeed = 0;
        }
        if (input.getBool("TestDrive")) {
            hands.setMotorPower("re", driveSpeed);
        }
        else {
            hands.setMotorPower("re", 0);
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
