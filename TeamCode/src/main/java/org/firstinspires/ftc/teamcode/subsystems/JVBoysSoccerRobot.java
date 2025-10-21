package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

import java.util.Arrays;
import java.util.List;

/**
 * JVBoysSoccerRobot is the robot base superclass.
 * All hardware and subsystems are initialized here.
 * GO JV BOYS SOCCER TEAM!
 */
public class JVBoysSoccerRobot {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    public BulkReading BR;
    private List<LynxModule> allHubs;
    private List<Subsystem> subsystems;
    public IMU imu;

    // Subsystems
    public Drivetrain drivetrainSubsystem;
    public Arm armSubsystem;
    public Claw clawSubsystem;

    // Hardware
    public DcMotorEx motorFL, motorFR, motorBL, motorBR; // mecanum motors when swerve doesn't work
    public DcMotorEx motorArmL, motorArmR;
    public Servo servoPivotL, servoPivotR;
    public Servo servoClawL;
    public Servo servoClawR;
    public Servo servoWrist;

    public DcMotorEx motorRigL, motorRigR;

    private int hertzCounter = 0;
    private double previousTime = 0;

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        // Configuring Hubs to auto mode for bulk reads
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initIMU();
        if (Arm.AUTO_NORESET_ARM_POSITION) {
            Arm.AUTO_NORESET_ARM_POSITION = false;
            initDrivetrainHardware();
            initClawHardware();
            initRiggingHardware();
            initArmHardware(false);
            Arm.armPresetIntakeSample = 4975; //
        }else {
            initHardware();
        }
        drivetrainSubsystem = new Drivetrain(hwMap, telemetry, this);

        if (RobotSettings.STORE_POSE) {
            drivetrainSubsystem.initYaw = RobotSettings.POSE_STORAGE;
            RobotSettings.STORE_POSE = false;
        }else {
            drivetrainSubsystem.resetInitYaw();
        }
        telemetry.addData("INIT YAW: ", drivetrainSubsystem.initYaw);

        clawSubsystem = new Claw(hwMap, telemetry, this);
        armSubsystem = new Arm(hwMap, telemetry, this);

        subsystems = Arrays.asList(drivetrainSubsystem, armSubsystem, clawSubsystem);
        BR = new BulkReading(this);

        Arm.DEFAULT_MAX_ACCELERATION = Arm.TELEOP_MAX_ACCELERATION;
        Arm.DEFAULT_MAX_VELOCITY = Arm.TELEOP_MAX_VELOCITY;
        Arm.DEFAULT_MAX_DECELERATION = Arm.TELEOP_MAX_DECELERATION;
    }

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry, boolean isAuto) {
        if (isAuto) {
            Arm.DEFAULT_MAX_ACCELERATION = Arm.AUTO_MAX_ACCELERATION;
            Arm.DEFAULT_MAX_VELOCITY = Arm.AUTO_MAX_VELOCITY;
            Arm.DEFAULT_MAX_DECELERATION = Arm.AUTO_MAX_DECELERATION;
            this.hwMap = hwMap;
            this.telemetry = telemetry;

            Arm.AUTO_NORESET_ARM_POSITION = true;
            RobotSettings.STORE_POSE = true;

            // Configuring Hubs to auto mode for bulk reads
            allHubs = hwMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            initIMU();
            initArmHardware(true);
            initClawHardware();
            clawSubsystem = new Claw(hwMap, telemetry, this);
            armSubsystem = new Arm(hwMap, telemetry, this);
            subsystems = Arrays.asList(clawSubsystem, armSubsystem);

            RobotSettings.POSE_STORAGE = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("PoseStorage: ", RobotSettings.POSE_STORAGE);
            BR = new BulkReading(this, true);
        }
    }

    public void initIMU() {
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters1 = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotSettings.LOGO_FACING_DIR, RobotSettings.USB_FACING_DIR));
        imu.initialize(parameters1);
    }

    public void initHardware() {
//        SwerveServoRight = hwMap.get(DcMotorSimple.class, "");
//        SwerveServoLeft = hwMap.get(DcMotorSimple.class, "");

        initDrivetrainHardware();
        initArmHardware(true);
        initClawHardware();
        initRiggingHardware();
    }

    public void initRiggingHardware() {
        motorRigL = hwMap.get(DcMotorEx.class, RobotSettings.RIGGING_LEFT);
        motorRigR = hwMap.get(DcMotorEx.class, RobotSettings.RIGGING_RIGHT);

        motorRigL.setDirection(RobotSettings.RIGGING_LEFT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorRigR.setDirection(RobotSettings.RIGGING_RIGHT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    public void initDrivetrainHardware() {
        motorFL = hwMap.get(DcMotorEx.class, RobotSettings.FL_NAME);
        motorBL = hwMap.get(DcMotorEx.class, RobotSettings.BL_NAME);
        motorFR = hwMap.get(DcMotorEx.class, RobotSettings.FR_NAME);
        motorBR = hwMap.get(DcMotorEx.class, RobotSettings.BR_NAME);

        motorFL.setDirection(RobotSettings.FL_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorFR.setDirection(RobotSettings.FR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorBR.setDirection(RobotSettings.BR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorBL.setDirection(RobotSettings.BL_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initArmHardware(boolean resetEncoder) {
        motorArmL = hwMap.get(DcMotorEx.class, RobotSettings.ARM_LMOTOR_NAME);
        motorArmR = hwMap.get(DcMotorEx.class, RobotSettings.ARM_RMOTOR_NAME);

        motorArmL.setDirection(RobotSettings.ARM_LMOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorArmR.setDirection(RobotSettings.ARM_RMOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        if (resetEncoder) {
            motorArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        motorArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initClawHardware() {
        servoPivotL = hwMap.servo.get(RobotSettings.ARM_LPIVOT_NAME);
        servoPivotR = hwMap.servo.get(RobotSettings.ARM_RPIVOT_NAME);
        servoPivotL.setDirection(RobotSettings.ARM_LPIVOT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servoPivotR.setDirection(RobotSettings.ARM_RPIVOT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

//        servoWristDiffyL = hwMap.servo.get(RobotSettings.CLAW_LWRIST_NAME);
//        servoWristDiffyR = hwMap.servo.get(RobotSettings.CLAW_RWRIST_NAME);
//        servoWristDiffyL.setDirection(RobotSettings.CLAW_LWRIST_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
//        servoWristDiffyR.setDirection(RobotSettings.CLAW_RWRIST_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        servoClawL = hwMap.servo.get(RobotSettings.CLAW_SERVOL_NAME);
        servoClawL.setDirection(RobotSettings.CLAW_SERVOL_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        servoClawR = hwMap.servo.get(RobotSettings.CLAW_SERVOR_NAME);
        servoClawR.setDirection(RobotSettings.CLAW_SERVOR_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        servoWrist = hwMap.servo.get(RobotSettings.CLAW_WRIST_SERVO);
        servoWrist.setDirection(RobotSettings.CLAW_WRIST_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public void addTelemetry() {
        if (UseTelemetry.ROBOT_TELEMETRY) {
            for (Subsystem s : subsystems) {
                s.addTelemetry();
            }
        }
    }
    public void update(boolean updateSubsystems, boolean useTelemetry) {
        if (updateSubsystems) {
            for (Subsystem s : subsystems) {
                s.update();
            }
        }
        if (useTelemetry) {
            addTelemetry();
        }
        hertzCounter++;
        if (RobotSettings.SUPER_TIME.seconds() - previousTime > 1.0) {
            telemetry.addData("HERTZ: ", hertzCounter);
            hertzCounter = 0;
        }
        previousTime = RobotSettings.SUPER_TIME.seconds();
        telemetry.update();
        BR.readAll();
    }
}
