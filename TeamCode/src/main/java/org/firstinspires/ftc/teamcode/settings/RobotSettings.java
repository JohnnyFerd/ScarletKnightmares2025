package org.firstinspires.ftc.teamcode.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class RobotSettings {
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static ElapsedTime SUPER_TIME = new ElapsedTime();

    public static double POSE_STORAGE = 0;
    public static double TOTAL_ENCODER_TICKS = 100;
    public static boolean STORE_POSE = false;

    // DRIVETRAIN

    public static final String FL_NAME = "FL";
    public static final String FR_NAME = "FR";
    public static final String BL_NAME = "BL";
    public static final String BR_NAME = "BR";
    public static final String SPIN_NAME = "spindexer";
//br-0, bl-1, fr-2, fl-3
    public static final boolean FL_REVERSED = true;
    public static final boolean FR_REVERSED = false;
    public static final boolean BL_REVERSED = true;
    public static final boolean BR_REVERSED = false;


    //SHOOTER

}
