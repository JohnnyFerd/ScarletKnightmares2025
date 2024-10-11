package org.firstinspires.ftc.teamcode.settings;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotSettings {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

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
    public static boolean ARM_LMOTOR_REVERSED = true;
    public static boolean ARM_RMOTOR_REVERSED = false;

}
