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

    public static int armPresetRest = -50; // FINAL
    public static int armPresetIntakeSpecimen = 1700; //
    public static int armPresetIntakeSample = 1800; //
    public static int armPresetDepositSpecimen = 1200; // maybe good?
    public static int armPreset1DepositSample = 425; //

    public static double pivotPresetRest = 0;
    public static double pivotPresetIntakeSpecimen = 0.78;
    public static double pivotPresetIntakeSample = 0.2;
    public static double pivotPresetDepositSpecimen = 0.46;
    public static double pivotPresetDepositSample = 0.9;
    public static double pivotDownIncrement = 0.35;

    public static int autoArmSpecimenPreset = 1700;
    public static double autoPivotSpecimenPreset = 0.78;

    public boolean pivotDown = false;
    public double previousPivotPos = 0;

    public static double pivotSpeedConstant = 0.005;
    public static double armSpeedConstant = 2.2;
    public static double armSpeedConstantBig = 4.4;

    public static double MAX_POWER = 1;
    public ElapsedTime motionProfileTime = new ElapsedTime();

    private double maxVelocity = 0;
    private int STARTING_POS = 0;
    private int ENDING_POS = 0;
    private double previousPower = 100000;
    private double previousRefPos = 100000;
    private double previousCurrentPos = 100000;

    private int counter = 0;

    public double referencePos = 0; // for the basic_pid state

    public enum ArmState {
        MOTION_PROFILE,
        BASIC_PID,
        AT_REST,
        NOTHING
    }
    public ArmState armState = ArmState.AT_REST;

    public Arm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.mp = new MotionProfile();
        pid = new PIDController();
    }

    public void setMotionProfile(int targetPosition) {
        noEncoders();
        motionProfileTime.reset();
        mp.setStartingTime(motionProfileTime.seconds());

        STARTING_POS = BulkReading.pMotorArmR;
        ENDING_POS = targetPosition;

        referencePos = targetPosition;

        mp.setProfile(STARTING_POS, ENDING_POS);

//        armState = ArmState.MOTION_PROFILE;
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
            telemetry.addData("    Arm Power", robot.motorArmL.getPower());
//            telemetry.addData("    Arm Encoder Position (L)", BulkReading.pMotorArmL);
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
                mp.updateState(motionProfileTime.seconds());
                double refPos = mp.getInstantPosition();
                double refVel = mp.getInstantVelocity();
                double refAcl = mp.getInstantAcceleration();

                if (UseTelemetry.ARM_TELEMETRY) {
                    telemetry.addData("    MP TIME", mp.getTimeElapsed());
                    telemetry.addData("    Reference Position", refPos);
                    telemetry.addData("    Arm Encoder Position (R)", BulkReading.pMotorArmR);
                    telemetry.addData("    Pivot Servo Position", robot.servoPivotR.getPosition());
                    telemetry.update();
//                    telemetry.addData("    Reference Velocity", refVel);
//                    telemetry.addData("    Reference Acceleration", refAcl);
                }

                if (mp.getTimeElapsed() > (mp.getEntireMPTime() / 2.0)) {
                    pivotQueue();
                }

                double pidPower = 0;
                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousRefPos == refPos) ) {
                    pidPower = pid.calculatePID(refPos, BulkReading.pMotorArmR);
                    setArmPower(pidPower);
                }

                previousCurrentPos = BulkReading.pMotorArmR;
                previousRefPos = refPos;

//                if (!mp.isBusy()) {
//                    armState = ArmState.BASIC_PID;
//                }
                break;
            case BASIC_PID:
//                if ( !(previousCurrentPos == BulkReading.pMotorArmR && previousRefPos == referencePos) ) {
                    double power = pid.calculatePID(referencePos, BulkReading.pMotorArmR);
                    setArmPower(power);
//                }
//                previousCurrentPos = BulkReading.pMotorArmR;
//                previousRefPos = referencePos;
                break;
            case NOTHING:
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
            counter = 1;
        }else {
            setPivotIntakeSpecimen();
            counter = 0;
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
            counter = 2;
        }else {
            setPivotIntakeSample();
            counter = 0;
        }
        setMotionProfile(armPresetIntakeSample);
    }
    public void setDepositSpecimen(boolean pivotTimed) {
        if (pivotTimed) {
            counter = 3;
        }else {
            setPivotDepositSpecimen();
            counter = 0;
        }
        setMotionProfile(armPresetDepositSpecimen);
    }
    public void setDepositSample(boolean pivotTimed) {
        if (pivotTimed) {
            counter = 4;
        }else {
            setPivotDepositSample();
            counter = 0;
        }
        setMotionProfile(armPreset1DepositSample);
    }

    public void pivotQueue() {
        switch (counter) {
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
        counter = 0;
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
