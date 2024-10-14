package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;

@Config
public class Rigging extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private double previousPower = -2;

    public static double RIG_LBOTTOM = 0;
    public static double RIG_RBOTTOM = 0;
    public static double RIG_LTOP = 0.5;
    public static double RIG_RTOP = 0.5;

    public enum RiggingState {
        ARMS_RESTING,
        ARMS_UP,
        SERVOS_OFF,
        NOTHING
    }

    public RiggingState riggingState = RiggingState.NOTHING;

    public Rigging(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.ARM_TELEMETRY) {
            telemetry.addLine("RIGGING TELEMETRY: ON");
//            telemetry.addLine("ARM TELEMETRY: ON");

        }else {
            telemetry.addLine("RIGGING TELEMETRY: OFF");
        }
    }

    @Override
    public void update() {
        switch (riggingState) {
            case ARMS_RESTING:
                armsDown();
                break;
            case ARMS_UP:
                armsUp();
                break;
            case SERVOS_OFF:
                robot.servoRigL.getController().pwmDisable();
                robot.servoRigR.getController().pwmDisable();
                break;
            case NOTHING:
                break;
        }
    }

    public void armsUp() {
        robot.servoRigL.setPosition(RIG_LTOP);
        robot.servoRigR.setPosition(RIG_RTOP);
    }

    public void armsDown() {
        robot.servoRigL.setPosition(RIG_LBOTTOM);
        robot.servoRigR.setPosition(RIG_RBOTTOM);
    }

    @Override
    public void stop() {

    }
}
