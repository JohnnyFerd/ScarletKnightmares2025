package org.firstinspires.ftc.teamcode.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class RobotSettings {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static ElapsedTime SUPER_TIME = new ElapsedTime();

    public static int ARM_VERTICAL_THRESHOLD = 2750;
    public static double RIGGING_POWER = 0.6;
    public static double RIGGING_FEEDFORWARD = 0.25;

    // DRIVETRAIN

    public static final String FL_NAME = "FL";
    public static final String FR_NAME = "FR";
    public static final String BL_NAME = "BL";
    public static final String BR_NAME = "BR";

    public static final boolean FL_REVERSED = true;
    public static final boolean FR_REVERSED = false;
    public static final boolean BL_REVERSED = true;
    public static final boolean BR_REVERSED = false;

    public static final String ARM_LMOTOR_NAME = "ArmLeft";
    public static final String ARM_RMOTOR_NAME = "ArmRight";
    public static final boolean ARM_LMOTOR_REVERSED = true;
    public static final boolean ARM_RMOTOR_REVERSED = false;

    public static final String ARM_LPIVOT_NAME = "ArmPivotLeft";
    public static final String ARM_RPIVOT_NAME = "ArmPivotRight";
    public static boolean ARM_LPIVOT_REVERSED = false;
    public static boolean ARM_RPIVOT_REVERSED = true;

    public static final String CLAW_SERVOL_NAME = "ClawServoL";
    public static boolean CLAW_SERVOL_REVERSED = false;
    public static final String CLAW_SERVOR_NAME = "ClawServoR";
    public static boolean CLAW_SERVOR_REVERSED = true;

    public static final String CLAW_WRIST_SERVO = "ClawWrist";
    public static boolean CLAW_WRIST_REVERSED = false;

    public static final String RIGGING_LEFT = "RigLeft";
    public static final String RIGGING_RIGHT = "RigRight";
    public static boolean RIGGING_LEFT_REVERSED = false;
    public static boolean RIGGING_RIGHT_REVERSED = true;

    // ENCODER MAPPING
    public static final int TOTAL_ENCODER_TICKS = 8192;
    public static final double GEAR_RATIO = 1;
    public static double ENCODER_CONVERSION_CONSTANT = ((double)TOTAL_ENCODER_TICKS * GEAR_RATIO) / 360.0;

}
