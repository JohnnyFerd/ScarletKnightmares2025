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

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private MotionProfile mp;
    private ArmPIDController pid;

    // TODO: test downwards motion profile on the arm with extremely small acl, dcl, vel to see if there is still that unsmooth motion
    // TODO: tune the different pid values based on increments of distance

    public static int DEFAULT_MAX_VELOCITY = 3000; // enocder ticks per second
    public static int DEFAULT_MAX_ACCELERATION = 3000; // encoder ticks per second
    public static int DEFAULT_MAX_DECELERATION = 1000;

    private int currentMaxVel = 0;
    private int currentMaxAcl = 0;
    private int currentMaxDcl = 0;

    public static int armPresetRest = -80; //
    public static int armPresetIntakeSpecimen = 0; //
    public static int armPresetIntakeSample = 300; //
    public static int armPresetDepositSpecimen = 1500; //
    public static int armPreset1DepositSample = 2750; //

    public static double pivotPresetRest = 0;
    public static double pivotPresetIntakeSpecimen = 0.31;
    public static double pivotPresetIntakeSample = 0.45;
    public static double pivotPresetDepositSpecimen = 0.35;
    public static double pivotPresetDepositSample = 0.15;
    public static double pivotDownIncrement = 0.45;

    public static final int armPresetIntakeSpecimenGround = 0;
    public static final double pivotPresetIntakeSpecimenGround = 0.31;

    private boolean DEPOSIT_SAMPLE = false;

    public boolean pivotDown = false;
    public double previousPivotPos = 0;

    public static final double pivotSpeedConstant = 0.005;
    public static final double armSpeedConstant = 8;

    public static double MAX_POWER = 1;

    private double previousPower = 100000;
    private double previousInstantRefPos = 100000;
    private double previousCurrentPos = 100000;
    private double previousRefPos = 10000;
    private boolean fightingGravity = true;

    public int pivotCounter = 0;

    public double referencePos = 0; // for the basic_pid state

    public enum ArmState {
        MOTION_PROFILE,
        BASIC_PID,
        AT_REST
    }
    public ArmState armState = ArmState.AT_REST;

    public Arm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.mp = new MotionProfile(telemetry);
        pid = new ArmPIDController(robot);
    }

    // DEFAULT ASYMMETRIC PROFILE
    public void setMotionProfile(int targetPosition) {
        referencePos = targetPosition; // used for manual control later in teleop
        armState = ArmState.MOTION_PROFILE;

        currentMaxAcl = DEFAULT_MAX_ACCELERATION;
        currentMaxVel = DEFAULT_MAX_VELOCITY;
        currentMaxDcl = DEFAULT_MAX_DECELERATION;
    }
    public void setMotionProfile(int targetPosition, int acl, int vel, int dcl) {
        referencePos = targetPosition; // used for manual control later in teleop
        armState = ArmState.MOTION_PROFILE;

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

    /**
     * Changes PID constants for power calculation dynamically based on goal distance
     * @param reference
     * @param state
     * @return
     */
    public double dynamicPIDPower(double reference, double state) {
//        double distance = Math.abs(reference - state);
//        if (distance < 300) {
//
//        }else if (distance < 1000) {
//
//        }else if (distance < 2000) {
//
//        }else {
//
//        }
        return pid.calculatePID(reference, state, fightingGravity) + pid.calculateF(referencePos);
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
        switch(armState) {
            case MOTION_PROFILE:
                if (referencePos != previousRefPos) {
                    mp.setProfile(new MotionProfileParameters(BulkReading.pMotorArmR, (int)referencePos, currentMaxAcl, currentMaxVel, currentMaxDcl));
                    fightingGravity = true;
                    if (referencePos > 2750) {
                        if (BulkReading.pMotorArmR < referencePos) {
                            fightingGravity = false;
                        }
                    }else {
                        if (BulkReading.pMotorArmR > referencePos) {
                            fightingGravity = false;
                        }
                    }
                }

                mp.updateState();
                double instantRefPos = mp.getInstantPosition();

                if (mp.getTimeElapsed() > (mp.getEntireMPTime() / 2.0)) {
                    pivotQueue();
                }

                // TODO: test the deposit sample preset to see if it automatically extends the slides after MP completes
                if (DEPOSIT_SAMPLE) {
                    if (!mp.isBusy()) {
                        robot.slideSubsystem.referencePos = LinearSlide.slideMaxExtension;
                        DEPOSIT_SAMPLE = false;
                    }
                }

                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousInstantRefPos == instantRefPos) ) {
                    setArmPower(pid.calculatePID(instantRefPos, BulkReading.pMotorArmR, fightingGravity) + pid.calculateF(referencePos));
                }
                previousCurrentPos = BulkReading.pMotorArmR;
                previousInstantRefPos = instantRefPos;
                previousRefPos = referencePos;
                break;
            case BASIC_PID:
//                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousRefPos == referencePos) ) {
                if (referencePos != previousRefPos) {
                    fightingGravity = true;
                    if (referencePos > 2750) {
                        if (BulkReading.pMotorArmR < referencePos) {
                            fightingGravity = false;
                        }
                    }else {
                        if (BulkReading.pMotorArmR > referencePos) {
                            fightingGravity = false;
                        }
                    }
                }
                setArmPower(pid.calculatePID(referencePos, BulkReading.pMotorArmR, fightingGravity) + pid.calculateF(referencePos));
//                }
//                previousCurrentPos = BulkReading.pMotorArmR;
                previousRefPos = referencePos;
                break;
            case AT_REST:
                setArmPower(0);
                break;
        }
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
        DEPOSIT_SAMPLE = true;
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
