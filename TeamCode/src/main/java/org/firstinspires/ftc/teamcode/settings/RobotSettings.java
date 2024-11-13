package org.firstinspires.ftc.teamcode.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class RobotSettings {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // DRIVETRAIN

    public static String FL_NAME = "FL";
    public static String FR_NAME = "FR";
    public static String BL_NAME = "BL";
    public static String BR_NAME = "BR";

    public static boolean FL_REVERSED = true;
    public static boolean FR_REVERSED = false;
    public static boolean BL_REVERSED = true;
    public static boolean BR_REVERSED = false;

    public static String ARM_LMOTOR_NAME = "ArmLeft";
    public static String ARM_RMOTOR_NAME = "ArmRight";
    public static boolean ARM_LMOTOR_REVERSED = false;
    public static boolean ARM_RMOTOR_REVERSED = true;

    public static String ARM_LPIVOT_NAME = "ArmPivotLeft";
    public static String ARM_RPIVOT_NAME = "ArmPivotRight";
    public static boolean ARM_LPIVOT_REVERSED = true;
    public static boolean ARM_RPIVOT_REVERSED = false;

    public static boolean RIGGING_LMOTOR_REVERSED = true;
    public static boolean RIGGING_RMOTOR_REVERSED = false;
    public static String RIGGING_LMOTOR_NAME = "RigLeft";
    public static String RIGGING_RMOTOR_NAME = "RigRight";
    public static boolean RIGGING_LSERVO_REVERSED = true;
    public static boolean RIGGING_RSERVO_REVERSED = true;
    public static String RIGGING_LSERVO_NAME = "RigLeftS";
    public static String RIGGING_RSERVO_NAME = "RigRightS";
    public static double RIGGING_MOTOR_SPEED = 0.5;

    public static String CLAW_SERVO_NAME = "ClawServoL";
    public static boolean CLAW_SERVO_REVERSED = false;
    public static String CLAW_SERVO2_NAME = "ClawServoR";
    public static boolean CLAW_SERVO2_REVERSED = true;

}
