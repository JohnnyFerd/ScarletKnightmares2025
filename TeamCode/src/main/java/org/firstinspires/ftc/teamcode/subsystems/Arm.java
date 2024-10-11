package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

public class Arm extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private MotionProfile mp;

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
//                mp.updateState(motionProfileTime.seconds());
//                double refPos = mp.getInstantPosition();
//                double refVel = mp.getInstantVelocity();
//                double refAcl = mp.getInstantAcceleration();
//
//                telemetry.addData("MP TIME", motionProfileTime.seconds());
//                telemetry.addData("Reference Position", refPos);
//                telemetry.addData("Reference Velocity", refVel);
//                telemetry.addData("Reference Acceleration", refAcl);
//
//                double pidPower = 0;
//                double fullstate = 0;
//                double output = 0;
//                if ( !(previousCurrentPos == BulkReading.pArmLeftMotor && previousRefPos == refPos) ) {
//                    pidPower = superController.calculatePID(refPos, BulkReading.pArmLeftMotor);
//                    double f_g = superController.positionalFeedforward(refPos);
//                    double k_va = superController.kvkaFeedforward(refVel, refAcl);
//
//                    output = pidPower + fullstate + f_g + k_va; // PID + gravity positional feedforward + velocity and acceleration feedforward
//                    setArmPower(output);
//                }
//
//                previousCurrentPos = BulkReading.pArmLeftMotor;
//                previousRefPos = refPos;
                break;
            case NOTHING:
                break;
        }

    }

    @Override
    public void stop() {

    }
}
