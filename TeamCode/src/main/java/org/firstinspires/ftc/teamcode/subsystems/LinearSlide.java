package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class LinearSlide extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private MotionProfile mp;
    private PIDController pid;

    public enum SlideState {
        MOTION_PROFILE,
        BASIC_PID,
        AT_REST
    }
    public SlideState slideState = SlideState.AT_REST;

    public LinearSlide(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.mp = new MotionProfile(telemetry);
        pid = new PIDController();
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.SLIDE_TELEMETRY) {
            telemetry.addLine("SLIDE TELEMETRY: ON");
//            telemetry.addData("    Arm Power", robot.motorArmL.getPower());
//            telemetry.addData("    Arm Encoder Position (R)", BulkReading.pMotorArmR);
//            telemetry.addData("    Pivot Servo Position", robot.servoPivotR.getPosition());
        }else {
            telemetry.addLine("SLIDE TELEMETRY: OFF");
        }
    }

    @Override
    public void update() {
        switch (slideState) {
            case AT_REST:
                break;
        }
    }

    @Override
    public void stop() {

    }
}
