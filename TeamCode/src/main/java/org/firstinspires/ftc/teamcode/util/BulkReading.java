package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

/**
 * Made to bulk read all sensors (not to be confused with the bulk read feature of lynx modules)
 */

public class BulkReading {
    private JVBoysSoccerRobot robot;
    private boolean isAuto = false;

    public static double pFL = 0;
    public static double pFR = 0;
    public static double pBL = 0;
    public static double pBR = 0;

    public static double vFL = 0, vFR = 0, vBL = 0, vBR = 0;

    public BulkReading(JVBoysSoccerRobot robot) {
        this.robot = robot;
    }
    public BulkReading(JVBoysSoccerRobot robot, boolean isAuto) {
        this(robot);
        this.isAuto = isAuto;
        // test
    }

    public void readAll() {
        if (isAuto) {
        }else {
            pFL = robot.motorFL.getCurrentPosition();
            pFR = robot.motorFR.getCurrentPosition();
            pBL = robot.motorBL.getCurrentPosition();
            pBR = robot.motorBR.getCurrentPosition();

//            vFL = robot.motorFL.getVelocity();
//            vFR = robot.motorFR.getVelocity();
//            vBL = robot.motorBL.getVelocity();
//            vBR = robot.motorBR.getVelocity();
//
        }
    }
}
