package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestingSuperclass {

    private HardwareMap hwMap;
    private Telemetry telemetry;

    // MOTORS
    private static String motorName1 = "";
    private static String motorName2 = "";
    private static String motorName3 = "";
    private static String motorName4 = "";
    private static String motorName5 = "";
    private static String motorName6 = "";
    private static String motorName7 = "";
    private static String motorName8 = "";

    public static String[] motorNames = {motorName1, motorName2, motorName3, motorName4, motorName5, motorName6, motorName7, motorName8};

    private static double motorPower1 = 0;
    private static double motorPower2 = 0;
    private static double motorPower3 = 0;
    private static double motorPower4 = 0;
    private static double motorPower5 = 0;
    private static double motorPower6 = 0;
    private static double motorPower7 = 0;
    private static double motorPower8 = 0;

    public static double[] motorPowers = {motorPower1, motorPower2, motorPower3, motorPower4, motorPower5, motorPower6, motorPower7, motorPower8};

    public TestingSuperclass(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

}
