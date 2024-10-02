package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private List<LynxModule> allHubs;
    private List<Subsystem> subsystems;
    public IMU imu;

    // Subsystems

    // Hardware

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
        initHardware();
        drivetrainSubsystem = new Drivetrain(hwMap, telemetry, this);
        clawSubsystem = new Claw(hwMap, telemetry, this);
        armSubsystem = new Arm(hwMap, telemetry, this);
        riggingSubsystem = new Rigging(hwMap, telemetry, this);
        launcherSubsystem = new AirplaneLauncher(hwMap, telemetry, this);

        subsystems = Arrays.asList(drivetrainSubsystem, clawSubsystem, armSubsystem, riggingSubsystem, launcherSubsystem);
    }

    public void initIMU() {
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters1 = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotSettings.LOGO_FACING_DIR, RobotSettings.USB_FACING_DIR));
        imu.initialize(parameters1);
    }

    public void initHardware() {

    }

}
