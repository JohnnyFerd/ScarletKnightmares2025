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

    public static int MAX_VELOCITY = 5000; // enocder ticks per second
    public static int MAX_ACCELERATION = 6000; // encoder ticks per second
    public static int MAX_DECELERATION = 1800;

    public static int armPresetRest = -120; // FINAL
    public static int armPresetIntakeSpecimen = 4820; //
    public static int armPresetIntakeSample = 4900; //
    public static int armPresetDepositSpecimen = 3480; // maybe good?
    public static int armPreset1DepositSample = 3100; //

    public static double pivotPresetRest = 0;
    public static double pivotPresetIntakeSpecimen = 0.74;
    public static double pivotPresetIntakeSample = 0.4;
    public static double pivotPresetDepositSpecimen = 0.475;
    public static double pivotPresetDepositSample = 0.9;
    public static double pivotDownIncrement = 0.45;

    public static final int autoArmSpecimenPreset = 1870;
    public static final double autoPivotSpecimenPreset = 0.68;

    public boolean pivotDown = false;
    public double previousPivotPos = 0;

    public static final double pivotSpeedConstant = 0.005;
    public static final double armSpeedConstant = 8;
    public static final double armSpeedConstantBig = 16;

    public static double MAX_POWER = 1;

    private double previousPower = 100000;
    private double previousRefPos = 100000;
    private double previousCurrentPos = 100000;
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
        pid = new ArmPIDController();
    }

    // DEFAULT ASYMMETRIC PROFILE
    public void setMotionProfile(int targetPosition) {
        referencePos = targetPosition; // used for manual control later in teleop
        mp.setProfile(new MotionProfileParameters(BulkReading.pMotorArmR, targetPosition, MAX_ACCELERATION, MAX_VELOCITY, MAX_DECELERATION));
        armState = ArmState.MOTION_PROFILE;
    }
    public void setMotionProfile(int targetPosition, int acl, int vel) {
        referencePos = targetPosition; // used for manual control later in teleop
        mp.setProfile(new MotionProfileParameters(BulkReading.pMotorArmR, targetPosition, acl, vel));
        armState = ArmState.MOTION_PROFILE;
    }
    public void setMotionProfile(int targetPosition, int acl, int vel, int dcl) {
        referencePos = targetPosition; // used for manual control later in teleop
        mp.setProfile(new MotionProfileParameters(BulkReading.pMotorArmR, targetPosition, acl, vel, dcl));
        armState = ArmState.MOTION_PROFILE;
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
            setMotionProfile(goalPosition, 5000, 5000, 2300);
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
        return pid.calculatePID(reference, state, fightingGravity);
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
        switch(armState) {
            case MOTION_PROFILE:
                mp.updateState();
                double refPos = mp.getInstantPosition();

                if (mp.getTimeElapsed() > (mp.getEntireMPTime() / 2.0)) {
                    pivotQueue();
                }

                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousRefPos == refPos) ) {
                    setArmPower(dynamicPIDPower(refPos, BulkReading.pMotorArmR));
                }
                previousCurrentPos = BulkReading.pMotorArmR;
                previousRefPos = refPos;
                break;
            case BASIC_PID:
//                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousRefPos == referencePos) ) {

                if (referencePos != previousRefPos) {
                    fightingGravity = true;
                    if (referencePos > 2650) {
                        if (BulkReading.pMotorArmR < referencePos) {
                            fightingGravity = false;
                        }
                    }else {
                        if (BulkReading.pMotorArmR > referencePos) {
                            fightingGravity = false;
                        }
                    }
                }
                setArmPower(dynamicPIDPower(referencePos, BulkReading.pMotorArmR) + pid.calculateF(referencePos));
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
    public void setAutoIntakeSpecimen() {
        setMotionProfile(autoArmSpecimenPreset);
        robot.servoPivotL.setPosition(autoPivotSpecimenPreset);
        robot.servoPivotR.setPosition(autoPivotSpecimenPreset);
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

}
