package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;

@Config
public class Claw extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    public boolean opened = false;

    public static double CLAW_CLOSED_POSITION = 0.57;
    public static double CLAW_OPENED_POSITION = 1;

    public Claw(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void openClaw() {
        opened = true;
        robot.servoClaw.setPosition(CLAW_OPENED_POSITION);
    }
    public void closeClaw() {
        opened = false;
        robot.servoClaw.setPosition(CLAW_CLOSED_POSITION);
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

    }

    @Override
    public void stop() {

    }
}
