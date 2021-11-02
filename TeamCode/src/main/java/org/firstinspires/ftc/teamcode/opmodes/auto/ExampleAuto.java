package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;


@Autonomous
public class ExampleAuto extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    float [] omniValues = new float [4];
    int step = 1;
    ElapsedTime timer;
    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            //do literally nothing
        }

    }

    public void init() {
        FeatureManager.setIsOpModeRunning(true);

    }
    public void loop() {
        switch (step) {
            case(1):
                timer = new ElapsedTime();
                driver.resetEncoders();
                driver.driveRaw(-0.5f, 0.5f, 0.5f, -0.5f);
                step++;
                break;
            case (2):
                if (driver.backLeft.getCurrentPosition()>=10) {
                step++;
                driver.resetEncoders();
                }
                break;
            case (3):
                driver.driveRaw(-0.5f, 0.5f, 0.5f, -0.5f);
                if (driver.backLeft.getCurrentPosition()>=1.5) {
                    step++;
                    driver.resetEncoders();
                }
                break;
            case (4):
                driver.driveRaw(0.5f, 0.5f, 0.5f, 0.5f);
                step++;
                break;
            case (5):
                if (driver.backLeft.getCurrentPosition()>50) {
                    driver.stopDrive();
                    step++;
                    driver.resetEncoders();
                }
                break;
            case(8):
                telemetry.addLine("Autonomous Complete");
                telemetry.addData("time", timer.milliseconds());
                telemetry.addData("Step #", step);
                telemetry.update();
        }
    }

}

