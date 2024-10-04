package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.PIDController;

public class SwerveDriveWheel {

    private PIDController directionController;
    private DcMotorEx speedMotor;
    private CRServo directionServo;
    private double directionEncoderValue;
    private double directionPower;

    public SwerveDriveWheel(double p, double i, double d, DcMotorEx speedMotor, CRServo directionServo) {
        directionController = new PIDController(p, i, d);
        this.speedMotor = speedMotor;
        this.directionServo = directionServo;
    }

    /**
     * Method will be called over and over again in the main loop
     * @param setpoint
     */
    public void convergeOnDirection(double setpoint) {

        directionPower = directionController.calculatePID(setpoint, directionEncoderValue);
        directionServo.setPower(directionPower);



    }

}
