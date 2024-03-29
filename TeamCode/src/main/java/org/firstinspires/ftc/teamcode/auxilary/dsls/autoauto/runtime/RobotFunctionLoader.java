package org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime;

import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.ClearAllFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.ClearFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.CloseMotorsFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.DownScaleFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.DriveOmniFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.DriveRawFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.DriveVerticalFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.DriveWithVerticalFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetBoolFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetCaptionValueSeparatorFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetColorIntegerFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetFloatFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetGamepadOneInfoFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetGamepadTwoInfoFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetHorizontalTicksFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetItemSeparatorFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetKeyFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetMotorPowerFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetMsTransmissionIntervalFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetScaleFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetServoPositionFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetServoPowerFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.GetVerticalTicksFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.HasNewDataFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.IsAutoClearFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.IsMacroRunningFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.IsSpecialFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.IsSpecialOneFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.ReadDataFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.ResetEncodersFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.RunMacroFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.RunToPositionFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.RunUsingEncodersFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.RunWithOutEncodersFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetAutoClearFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetCaptionValueSeparatorFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetIsOpModeRunningFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetItemSeparatorFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetMotorPowerFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetMsTransmissionIntervalFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetServoPositionFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetServoPowerFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SetTargetPositionsFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.SpeakFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.StopDriveFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.StopMacroFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.UpScaleFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.UpdateFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.UpdateSensorManagerFunction;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.robotfunctions.UpdateTelemetryManagerFunction;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;
    
    public class RobotFunctionLoader {
    
        public static void loadFunctions(AutoautoRuntimeVariableScope scope, FeatureManager... managers) {
            org.firstinspires.ftc.teamcode.managers.FeatureManager manFeature = null;
            org.firstinspires.ftc.teamcode.managers.input.InputManager manInput = null;
            org.firstinspires.ftc.teamcode.managers.macro.MacroManager manMacro = null;
            org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager manManipulation = null;
            org.firstinspires.ftc.teamcode.managers.movement.MovementManager manMovement = null;
            org.firstinspires.ftc.teamcode.managers.sensor.SensorManager manSensor = null;
            org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager manTelemetry = null;
            for(FeatureManager f : managers) {
                if(f instanceof org.firstinspires.ftc.teamcode.managers.FeatureManager) manFeature = (org.firstinspires.ftc.teamcode.managers.FeatureManager)f;
                if(f instanceof org.firstinspires.ftc.teamcode.managers.input.InputManager) manInput = (org.firstinspires.ftc.teamcode.managers.input.InputManager)f;
                if(f instanceof org.firstinspires.ftc.teamcode.managers.macro.MacroManager) manMacro = (org.firstinspires.ftc.teamcode.managers.macro.MacroManager)f;
                if(f instanceof org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager) manManipulation = (org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager)f;
                if(f instanceof org.firstinspires.ftc.teamcode.managers.movement.MovementManager) manMovement = (org.firstinspires.ftc.teamcode.managers.movement.MovementManager)f;
                if(f instanceof org.firstinspires.ftc.teamcode.managers.sensor.SensorManager) manSensor = (org.firstinspires.ftc.teamcode.managers.sensor.SensorManager)f;
                if(f instanceof org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager) manTelemetry = (org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager)f;
            }
            scope.put("setIsOpModeRunning", new SetIsOpModeRunningFunction(manFeature));
            
            scope.put("getKey", new GetKeyFunction(manInput));
            scope.put("update", new UpdateFunction(manInput));
            scope.put("getBool", new GetBoolFunction(manInput));
            scope.put("getFloat", new GetFloatFunction(manInput));
            
            scope.put("runMacro", new RunMacroFunction(manMacro));
            scope.put("stopMacro", new StopMacroFunction(manMacro));
            scope.put("isMacroRunning", new IsMacroRunningFunction(manMacro));
            
            scope.put("setServoPosition", new SetServoPositionFunction(manManipulation));
            scope.put("setServoPower", new SetServoPowerFunction(manManipulation));
            scope.put("setMotorPower", new SetMotorPowerFunction(manManipulation));
            //scope.put("timeSetMotorPower", new TimeSetMotorPowerFunction(manManipulation));
            scope.put("getMotorPower", new GetMotorPowerFunction(manManipulation));
            scope.put("getServoPower", new GetServoPowerFunction(manManipulation));
            scope.put("getServoPosition", new GetServoPositionFunction(manManipulation));
            scope.put("resetEncoders", new ResetEncodersFunction(manManipulation));
            scope.put("runUsingEncoders", new RunUsingEncodersFunction(manManipulation));
            scope.put("closeMotors", new CloseMotorsFunction(manManipulation));
            
            scope.put("driveRaw", new DriveRawFunction(manMovement));
            scope.put("stopDrive", new StopDriveFunction(manMovement));
            //scope.put("holdUp", new HoldUpFunction(manMovement));
            //scope.put("rotate", new RotateFunction(manMovement));
            //scope.put("timeDriveRaw", new TimeDriveRawFunction(manMovement));
            //scope.put("timeDriveOmni", new TimeDriveOmniFunction(manMovement));
            //scope.put("setDirection", new SetDirectionFunction(manMovement));
            //scope.put("testDriveOmni", new TestDriveOmniFunction(manMovement));
            //scope.put("encoderDriveRaw", new EncoderDriveRawFunction(manMovement));
            //scope.put("encoderDriveOmni", new EncoderDriveOmniFunction(manMovement));
            scope.put("runToPosition", new RunToPositionFunction(manMovement));
            scope.put("runUsingEncoders", new RunUsingEncodersFunction(manMovement));
            scope.put("runWithOutEncoders", new RunWithOutEncodersFunction(manMovement));
            scope.put("setTargetPositions", new SetTargetPositionsFunction(manMovement));
            scope.put("driveVertical", new DriveVerticalFunction(manMovement));
            scope.put("driveOmni", new DriveOmniFunction(manMovement));
            scope.put("resetEncoders", new ResetEncodersFunction(manMovement));
            scope.put("getScale", new GetScaleFunction(manMovement));
            scope.put("upScale", new UpScaleFunction(manMovement));
            scope.put("downScale", new DownScaleFunction(manMovement));
            scope.put("driveWithVertical", new DriveWithVerticalFunction(manMovement));
            //scope.put("flGetTicks", new FlGetTicksFunction(manMovement));
            //scope.put("frGetTicks", new FrGetTicksFunction(manMovement));
            //scope.put("blGetTicks", new BlGetTicksFunction(manMovement));
            //scope.put("brGetTicks", new BrGetTicksFunction(manMovement));
            scope.put("getHorizontalTicks", new GetHorizontalTicksFunction(manMovement));
            scope.put("getVerticalTicks", new GetVerticalTicksFunction(manMovement));
            
            scope.put("updateSensorManager", new UpdateSensorManagerFunction(manSensor));
            scope.put("getColorInteger", new GetColorIntegerFunction(manSensor));
            scope.put("isSpecial", new IsSpecialFunction(manSensor));
            scope.put("isSpecialOne", new IsSpecialOneFunction(manSensor));
            
            scope.put("clear", new ClearFunction(manTelemetry));
            scope.put("clearAll", new ClearAllFunction(manTelemetry));
            scope.put("speak", new SpeakFunction(manTelemetry));
            scope.put("updateTelemetryManager", new UpdateTelemetryManagerFunction(manTelemetry));
            scope.put("isAutoClear", new IsAutoClearFunction(manTelemetry));
            scope.put("setAutoClear", new SetAutoClearFunction(manTelemetry));
            scope.put("getMsTransmissionInterval", new GetMsTransmissionIntervalFunction(manTelemetry));
            scope.put("setMsTransmissionInterval", new SetMsTransmissionIntervalFunction(manTelemetry));
            scope.put("getItemSeparator", new GetItemSeparatorFunction(manTelemetry));
            scope.put("setItemSeparator", new SetItemSeparatorFunction(manTelemetry));
            scope.put("getCaptionValueSeparator", new GetCaptionValueSeparatorFunction(manTelemetry));
            scope.put("setCaptionValueSeparator", new SetCaptionValueSeparatorFunction(manTelemetry));
            scope.put("readData", new ReadDataFunction(manTelemetry));
            scope.put("hasNewData", new HasNewDataFunction(manTelemetry));
            scope.put("getGamepadOneInfo", new GetGamepadOneInfoFunction(manTelemetry));
            scope.put("getGamepadTwoInfo", new GetGamepadTwoInfoFunction(manTelemetry));
            
        }
    }
    