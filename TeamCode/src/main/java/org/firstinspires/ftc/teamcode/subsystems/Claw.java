package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@Config
public class Claw extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    public boolean leftOpened = false;
    public boolean rightOpened = false;
    private int thresholdEncoderValue = 1000;
    private boolean reversedControls = false;

    public static double CLAW_CLOSED_POSITIONL = 0.23;
    public static double CLAW_OPENED_POSITIONL = 0.45;
    public static double CLAW_CLOSED_POSITIONR = 0.15;
    public static double CLAW_OPENED_POSITIONR = 0.39;

    public Claw(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void openBothClaw() {
        leftOpened = true;
        rightOpened = true;
        robot.servoClawL.setPosition(CLAW_OPENED_POSITIONL);
        robot.servoClawR.setPosition(CLAW_OPENED_POSITIONR);
    }

    public void openLeftClaw() {
        if (reversedControls) {
            leftOpened = false;
            rightOpened = true;
            robot.servoClawL.setPosition(CLAW_CLOSED_POSITIONL);
            robot.servoClawR.setPosition(CLAW_OPENED_POSITIONR);
        }else {
            leftOpened = true;
            rightOpened = false;
            robot.servoClawL.setPosition(CLAW_OPENED_POSITIONL);
            robot.servoClawR.setPosition(CLAW_CLOSED_POSITIONR);
        }
    }

    public void openRightClaw() {
        if (reversedControls) {
            leftOpened = true;
            rightOpened = false;
            robot.servoClawL.setPosition(CLAW_OPENED_POSITIONL);
            robot.servoClawR.setPosition(CLAW_CLOSED_POSITIONR);
        }else {
            leftOpened = false;
            rightOpened = true;
            robot.servoClawL.setPosition(CLAW_CLOSED_POSITIONL);
            robot.servoClawR.setPosition(CLAW_OPENED_POSITIONR);
        }
    }

    public void closeBothClaw() {
        leftOpened = false;
        rightOpened = false;
        robot.servoClawL.setPosition(CLAW_CLOSED_POSITIONL);
        robot.servoClawR.setPosition(CLAW_CLOSED_POSITIONR);
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.CLAW_TELEMETRY) {
            telemetry.addLine("CLAW TELEMETRY: ON");
//            telemetry.addLine("ARM TELEMETRY: ON");

        }else {
            telemetry.addLine("CLAW TELEMETRY: OFF");
        }
    }

    @Override
    public void update() {
        if (BulkReading.pMotorArmR > thresholdEncoderValue) {
            reversedControls = true;
        }else {
            reversedControls = false;
        }
    }

    @Override
    public void stop() {

    }
}
