package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Arm extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private MotionProfile mp;
    private PIDController pid;

    public static double Kp = 0, Ki = 0, Kd = 0;

    public static double MAX_POWER = 0.5;
    public ElapsedTime motionProfileTime = new ElapsedTime();

    private double maxVelocity = 0;
    private int STARTING_POS = 0;
    private int ENDING_POS = 0;
    private double previousPower = 5;
    private double previousRefPos = 100000;
    private double previousCurrentPos = 100000;

    public enum ArmState {
        MOTION_PROFILE,
        AT_REST,
        NOTHING
    }

    public ArmState armState = ArmState.NOTHING;

    public Arm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.mp = new MotionProfile();
        pid = new PIDController(Kp, Ki, Kd);
    }

    public void setMotionProfile(int targetPosition) {
        noEncoders();
        motionProfileTime.reset();
        mp.setStartingTime(motionProfileTime.seconds());

        STARTING_POS = BulkReading.pMotorArmL;
        ENDING_POS = targetPosition;

        mp.setProfile(STARTING_POS, ENDING_POS);
    }

    public void noEncoders() {
        robot.motorArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public MotionProfile getMP() {
        return mp;
    }

    public void setArmPower(double power) {
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }
        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }

        if (power != previousPower) {
            robot.motorArmL.setPower(power);
            robot.motorArmR.setPower(power);
        }

        previousPower = power;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.ARM_TELEMETRY) {
            telemetry.addLine("ARM TELEMETRY: ON");
//            telemetry.addLine("ARM TELEMETRY: ON");

        }else {
            telemetry.addLine("ARM TELEMETRY: OFF");
        }
    }

    @Override
    public void update() {
        switch(armState) {
            case MOTION_PROFILE:
                mp.updateState(motionProfileTime.seconds());
                double refPos = mp.getInstantPosition();
                double refVel = mp.getInstantVelocity();
                double refAcl = mp.getInstantAcceleration();

                telemetry.addData("MP TIME", motionProfileTime.seconds());
                telemetry.addData("Reference Position", refPos);
                telemetry.addData("Reference Velocity", refVel);
                telemetry.addData("Reference Acceleration", refAcl);

                double pidPower = 0;
                double output = 0;
                if ( !(previousCurrentPos == BulkReading.pMotorArmL && previousRefPos == refPos) ) {
                    pidPower = pid.calculatePID(refPos, BulkReading.pMotorArmL);

                    output = pidPower;
                    setArmPower(output);
                }

                previousCurrentPos = BulkReading.pMotorArmL;
                previousRefPos = refPos;
                break;
            case NOTHING:
                break;
        }

    }

    @Override
    public void stop() {

    }
}
