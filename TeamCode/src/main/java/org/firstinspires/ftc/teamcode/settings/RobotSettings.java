package org.firstinspires.ftc.teamcode.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class RobotSettings {
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static ElapsedTime SUPER_TIME = new ElapsedTime();

    public static double POSE_STORAGE = 0;
    public static boolean STORE_POSE = false;

    // DRIVETRAIN

    public static final String FL_NAME = "FL";
    public static final String FR_NAME = "FR";
    public static final String BL_NAME = "BL";
    public static final String BR_NAME = "BR";

    public static final boolean FL_REVERSED = true;
    public static final boolean FR_REVERSED = false;
    public static final boolean BL_REVERSED = true;
    public static final boolean BR_REVERSED = false;


    //SHOOTER
    public static final String SHOOTER1_NAME = "shooter1";
    public static final boolean SHOOTER1_REVERSED = true;
    public static final String SHOOTER2_NAME = "shooter2";
    public static final boolean SHOOTER2_REVERSED = true;
    public static final String SHOOTER1_SERVO_NAME = "shooter1Servo";
    public static final String SHOOTER2_SERVO_NAME = "shooter2Servo";
    public static final String PADDLE1_NAME = "paddle1";
    public static final String PADDLE2_NAME = "paddle2";
}
