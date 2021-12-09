package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class EmergencyParkngCode extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    float[] omniValues = new float[4];
    int step = 1;
    ElapsedTime timer = new ElapsedTime();

    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            //do nothing
        }


    }

    public void init() {
        FeatureManager.setIsOpModeRunning(true);
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        /*DcMotor dw = hardwareMap.get(DcMotor.class, "dw"); todo uncomment this */
        hands = new ManipulationManager(
                new CRServo[]{},
                new String[]{},
                new Servo[]{},
                new String[]{},
                new DcMotor[]{fl, fr, br, bl, /*dw*/}, // todo uncomment this
                new String[]{"fl", "fr", "br", "bl", /*"dw"*/} //todo uncomment this
        );
        driver = new MovementManager(fl, fr, br, bl);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        driver.setDirection();
        timer = new ElapsedTime();
    }

    public void loop() {
        switch (step) {
            case (1):
                driver.timeDriveRaw(1500, 1f, 1f, 1f, 1f);

                step++;
                break;
            case (2):

                break;

        }

    }
}

