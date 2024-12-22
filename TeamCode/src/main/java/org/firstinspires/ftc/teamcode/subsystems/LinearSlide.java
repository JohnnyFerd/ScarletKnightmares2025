package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.LinkagePIDController;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.MotionProfileParameters;

@Config
public class LinearSlide extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private MotionProfile mp;
    private LinkagePIDController pid;

    public static int DEFAULT_MAX_VELOCITY = 300; // enocder ticks per second
    public static int DEFAULT_MAX_ACCELERATION = 300; // encoder ticks per second
    public static int DEFAULT_MAX_DECELERATION = 100;

    private int currentMaxVel = 0;
    private int currentMaxAcl = 0;
    private int currentMaxDcl = 0;

    private double previousPower = 0;
    public double referencePos = 0;
    public static double MAX_POWER = 0.5;

    private double previousInstantRefPos = 100000;
    private double previousCurrentPos = 100000;
    private double previousRefPos = 10000;

    public static int slideMaxExtension = 850;
    public static int slideDeExtension = 0;

    public static double slideSpeedConstant = 3;

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
        pid = new LinkagePIDController(telemetry);
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.SLIDE_TELEMETRY) {
            telemetry.addLine("SLIDE TELEMETRY: ON");
            telemetry.addData("    Slide Power", robot.motorSlide.getPower());
//            telemetry.addData("    Arm Encoder Position (R)", BulkReading.pMotorLinkage);
//            telemetry.addData("    Pivot Servo Position", robot.servoPivotR.getPosition());
        }else {
            telemetry.addLine("SLIDE TELEMETRY: OFF");
        }
    }

    @Override
    public void update() {
        switch (slideState) {
            case MOTION_PROFILE:
                if (referencePos != previousRefPos) {
                    mp.setProfile(new MotionProfileParameters(BulkReading.pMotorArmR, (int)referencePos, currentMaxAcl, currentMaxVel, currentMaxDcl));
                }

                mp.updateState();
                double instantRefPos = mp.getInstantPosition();

                if ( !(previousCurrentPos == BulkReading.pMotorLinkage && previousRefPos == instantRefPos) ) {
                    setSlidePower(pid.calculatePID(referencePos, BulkReading.pMotorLinkage, BulkReading.pMotorArmR) + pid.calculateF(BulkReading.pMotorArmR));
                }
                previousCurrentPos = BulkReading.pMotorLinkage;
                previousInstantRefPos = instantRefPos;
                previousRefPos = referencePos;
                break;
            case BASIC_PID:
                double pow = pid.calculatePID(referencePos, BulkReading.pMotorLinkage, BulkReading.pMotorArmR) + pid.calculateF(BulkReading.pMotorArmR);
                setSlidePower(pow);
                break;
            case AT_REST:
                setSlidePower(0);
                break;
        }
    }

    @Override
    public void stop() {

    }

    // DEFAULT ASYMMETRIC PROFILE
    public void setMotionProfile(int targetPosition) {
        referencePos = targetPosition; // used for manual control later in teleop
        slideState = SlideState.MOTION_PROFILE;

        currentMaxAcl = DEFAULT_MAX_ACCELERATION;
        currentMaxVel = DEFAULT_MAX_VELOCITY;
        currentMaxDcl = DEFAULT_MAX_DECELERATION;
    }
    public void setMotionProfile(int targetPosition, int acl, int vel, int dcl) {
        referencePos = targetPosition; // used for manual control later in teleop
        slideState = SlideState.MOTION_PROFILE;

        currentMaxAcl = acl;
        currentMaxVel = vel;
        currentMaxDcl = dcl;
    }

    public void setSlidePower(double power) {
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }
        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }
        if (power != previousPower) {
            robot.motorSlide.setPower(power);
        }
        previousPower = power;
    }
}
