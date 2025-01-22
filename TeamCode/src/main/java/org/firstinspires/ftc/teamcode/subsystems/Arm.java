package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.MotionProfileParameters;
import org.firstinspires.ftc.teamcode.util.ArmPIDController;

@Config
public class Arm extends Subsystem {

    // TODO: make a large and small PID similar to Pedro Pathing, make it so if value is the same for 10 loops, switch to small pid and be able to check if this works

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private MotionProfile mp;
    private ArmPIDController pid;

    public static int DEFAULT_MAX_VELOCITY = 12000; // enocder ticks per second
    public static int DEFAULT_MAX_ACCELERATION = 12000; // encoder ticks per second
    public static int DEFAULT_MAX_DECELERATION =  1700;

    private int currentMaxVel = 0;
    private int currentMaxAcl = 0;
    private int currentMaxDcl = 0;

    public static int armPresetRest = -20; //
//    public static int armPresetIntakeSpecimen = 4810;
    public static int armPresetIntakeSpecimen = 300; //

    public static int armLowerConstantSample = 250;
    public static int armLowerConstantSpecimen = 250;

    public static int armPresetIntakeSample = 4550; //
    
    public static int armPresetDepositSpecimen = 3350; //
    public static int armPresetDepositSpecimenAuto = 2900;
    public static int armPreset1DepositSample = 2750; //

    public static double pivotPresetRest = 1.00;
    public static double pivotPresetIntakeSpecimen = 0.657;
    public static double pivotPresetIntakeSample = 0.88;
    public static double pivotPresetDepositSpecimen = 0.566;
    public static double pivotPresetDepositSample = 0.4;

    public static final int armPresetIntakeSpecimenGround = 0;
    public static final double pivotPresetIntakeSpecimenGround = 0.31;

    public double previousPivotPos = 0;

    public static final double pivotSpeedConstant = 0.012;
    public static final double wristSpeedConstant = 0.008;
    public static final double armSpeedConstant = 8;

    public static double MAX_POWER = 1;

    private double previousPower = 100000;
    private double previousInstantRefPos = 100000;
    private double previousCurrentPos = 100000;
    private double previousRefPos = 10000;

    public int pivotCounter = 0;

    public static double referencePos = 0; // for the basic_pid state
    
    public static double PID_ENCODER_DISTANCE_THRESHOLD = 100; // if within 100 ticks, switch to small pid
    public static double PID_LOOP_THRESHOLD = 10; // if state has been the same for 10 loops, assume its reached steady state and move to small pid
    public static boolean PID_BIG = true;
    public static boolean TWO_PID = true;
    private int pidCounter = 0;

    public enum ArmState {
        MOTION_PROFILE,
        BASIC_PID,
        AT_REST
    }
    public ArmState armState = ArmState.AT_REST;
    private ArmState previousArmState = ArmState.AT_REST;

    public Arm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.mp = new MotionProfile(telemetry);
        pid = new ArmPIDController(robot, telemetry);
    }

    // DEFAULT ASYMMETRIC PROFILE
    public void setMotionProfile(int targetPosition) {
        referencePos = targetPosition; // used for manual control later in teleop
        armState = ArmState.MOTION_PROFILE;
        this.mp.setBusy(true);

        currentMaxAcl = DEFAULT_MAX_ACCELERATION;
        currentMaxVel = DEFAULT_MAX_VELOCITY;
        currentMaxDcl = DEFAULT_MAX_DECELERATION;
    }
    public void setMotionProfile(int targetPosition, int acl, int vel, int dcl) {
        referencePos = targetPosition; // used for manual control later in teleop
        armState = ArmState.MOTION_PROFILE;
        this.mp.setBusy(true);

        currentMaxAcl = acl;
        currentMaxVel = vel;
        currentMaxDcl = dcl;
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

    public void setSmartArmPower(int goalPosition) {
        int distance = Math.abs(goalPosition - BulkReading.pMotorArmR);
        if (distance < 300) {
            referencePos = goalPosition;
            armState = ArmState.BASIC_PID;
        }
        else if (distance < 1000) {
//            setMotionProfile(goalPosition, 5000, 5000, 2300);
        }
        else {
            setMotionProfile(goalPosition);
        }
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.MOTION_PROFILE_TELEMETRY) {
            mp.addTelemetry();
        }else {
            telemetry.addLine("MOTION PROFILE TELEMETRY: OFF");
        }
        if (UseTelemetry.ARM_TELEMETRY) {
            telemetry.addLine("ARM TELEMETRY: ON");
            telemetry.addData("    Arm Power", robot.motorArmL.getPower());
            telemetry.addData("    Arm Encoder Position (R)", BulkReading.pMotorArmR);
            telemetry.addData("    Pivot Servo Position", robot.servoPivotR.getPosition());
            telemetry.addData("    Wrist Servo Position", robot.servoWrist.getPosition());
        }else {
            telemetry.addLine("ARM TELEMETRY: OFF");
        }
    }

    @Override
    public void update() {
//        if (referencePos > 8000) {
//            referencePos = 8000;
//        }
//        if (referencePos < -1000) {
//            referencePos = -1000;
//        }
        if (previousArmState != armState) { // idk if this is needed bc of how negligible the ticks are but whatever
            PID_BIG = true;
            pidCounter = 0;
        }
        switch(armState) {
            case MOTION_PROFILE:
                if (referencePos != previousRefPos) {
                    mp.setProfile(new MotionProfileParameters(BulkReading.pMotorArmR, (int)referencePos, currentMaxAcl, currentMaxVel, currentMaxDcl));
                }

                mp.updateState();
                double instantRefPos = mp.getInstantPosition();

                if (mp.getTimeElapsed() > (mp.getEntireMPTime() / 2.0)) {
                    pivotQueue();
                }

                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousInstantRefPos == instantRefPos) ) {
                    double ffPower = pid.calculateF(referencePos);
                    if (TWO_PID) {
                        if (instantRefPos != previousInstantRefPos) {
                            PID_BIG = true;
                            pidCounter = 0;
                            // every time we change reference positions, we assume were doing big pid first
                        }

                        if (PID_BIG) {
                            setArmPower(ffPower + pid.calculatePIDBig(referencePos, BulkReading.pMotorArmR));
                            // currently assuming big pid at first
                            if (previousCurrentPos == BulkReading.pMotorArmR) {
                                // only if we have 10 consecutive same pos, we assume it converged
                                pidCounter++;
                            }else {
                                pidCounter = 0;
                            }
                            if (pidCounter > PID_LOOP_THRESHOLD) { // assume converged, switch to small pid
                                PID_BIG = false;
                                pidCounter = 0;
                            }
                        }else {
                            setArmPower(ffPower + pid.calculatePIDSmall(referencePos, BulkReading.pMotorArmR));
                        }
                    }else {
                        setArmPower(ffPower + pid.calculatePIDBig(referencePos, BulkReading.pMotorArmR));
                    }
                }
                previousCurrentPos = BulkReading.pMotorArmR;
                previousInstantRefPos = instantRefPos;
                previousRefPos = referencePos;
                break;
            case BASIC_PID:
                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousRefPos == referencePos) ) {
                    double ffPower = pid.calculateF(referencePos);
                    if (TWO_PID) {
                        if (referencePos != previousInstantRefPos) {
                            PID_BIG = true;
                            pidCounter = 0;
                            // every time we change reference positions, we assume were doing big pid first
                        }

                        if (PID_BIG) {
                            setArmPower(ffPower + pid.calculatePIDBig(referencePos, BulkReading.pMotorArmR));
                            // currently assuming big pid at first
                            if (previousCurrentPos == BulkReading.pMotorArmR) {
                                // only if we have 10 consecutive same pos, we assume it converged
                                pidCounter++;
                            }else {
                                pidCounter = 0;
                            }
                            if (pidCounter > PID_LOOP_THRESHOLD) { // assume converged, switch to small pid
                                PID_BIG = false;
                                pidCounter = 0;
                            }
                        }else {
                            setArmPower(ffPower + pid.calculatePIDSmall(referencePos, BulkReading.pMotorArmR));
                        }
                    }else {
                        setArmPower(ffPower + pid.calculatePIDBig(referencePos, BulkReading.pMotorArmR));
                    }
                }
                previousCurrentPos = BulkReading.pMotorArmR;
                previousRefPos = referencePos;
                break;
            case AT_REST:
                setArmPower(0);
                break;
        }
        previousArmState = armState;
    }

    @Override
    public void stop() {

    }

    public void setRest() {
        setPivotRest();
        setMotionProfile(armPresetRest);
    }

    public void setIntakeSpecimen(boolean pivotTimed) {
        if (pivotTimed) {
            pivotCounter = 1;
        }else {
            setPivotIntakeSpecimen();
            pivotCounter = 0;
        }
        setMotionProfile(armPresetIntakeSpecimen);
    }
    public void setIntakeSpecimenGround() {
        setMotionProfile(armPresetIntakeSpecimenGround);
        robot.servoPivotL.setPosition(pivotPresetIntakeSpecimenGround);
        robot.servoPivotR.setPosition(pivotPresetIntakeSpecimenGround);
    }
    public void setIntakeSample(boolean pivotTimed) {
        if (pivotTimed) {
            pivotCounter = 2;
        }else {
            setPivotIntakeSample();
            pivotCounter = 0;
        }
        setMotionProfile(armPresetIntakeSample);
    }
    public void setDepositSpecimen(boolean pivotTimed) {
        if (pivotTimed) {
            pivotCounter = 3;
        }else {
            setPivotDepositSpecimen();
            pivotCounter = 0;
        }
        setMotionProfile(armPresetDepositSpecimen);
    }
    public void setDepositSample(boolean pivotTimed) {
        if (pivotTimed) {
            pivotCounter = 4;
        }else {
            setPivotDepositSample();
            pivotCounter = 0;
        }
        setMotionProfile(armPreset1DepositSample);
    }

    public void pivotQueue() {
        switch (pivotCounter) {
            case 1:
                setPivotIntakeSpecimen();
                break;
            case 2:
                setPivotIntakeSample();
                break;
            case 3:
                setPivotDepositSpecimen();
                break;
            case 4:
                setPivotDepositSample();
                break;
        }
        pivotCounter = 0;
    }

    public void setPivotRest() {
        robot.servoPivotL.setPosition(pivotPresetRest);
        robot.servoPivotR.setPosition(pivotPresetRest);
        previousPivotPos = pivotPresetRest;
    }
    public void setPivotIntakeSpecimen() {
        robot.servoPivotL.setPosition(pivotPresetIntakeSpecimen);
        robot.servoPivotR.setPosition(pivotPresetIntakeSpecimen);
        previousPivotPos = pivotPresetIntakeSpecimen;
    }
    public void setPivotIntakeSample() {
        robot.servoPivotL.setPosition(pivotPresetIntakeSample);
        robot.servoPivotR.setPosition(pivotPresetIntakeSample);
        previousPivotPos = pivotPresetIntakeSample;
    }
    public void setPivotDepositSpecimen() {
        robot.servoPivotL.setPosition(pivotPresetDepositSpecimen);
        robot.servoPivotR.setPosition(pivotPresetDepositSpecimen);
        previousPivotPos = pivotPresetDepositSpecimen;
    }
    public void setPivotDepositSample() {
        robot.servoPivotL.setPosition(pivotPresetDepositSample);
        robot.servoPivotR.setPosition(pivotPresetDepositSample);
        previousPivotPos = pivotPresetDepositSample;
    }


    public void setPivot(double position) {
        robot.servoPivotL.setPosition(position);
        robot.servoPivotR.setPosition(position);
    }
    public void setLeftPivot(double position) {
        robot.servoPivotL.setPosition(position);
    }
    public void setRightPivot(double position) {
        robot.servoPivotR.setPosition(position);
    }

}
