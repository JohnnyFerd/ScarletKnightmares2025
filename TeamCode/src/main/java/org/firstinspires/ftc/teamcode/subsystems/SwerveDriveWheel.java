//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.util.PIDController;
//
//public class SwerveDriveWheel {
//
//    private PIDController directionController;
//    private DcMotorEx speedMotor;
//    private CRServo directionServo;
//    private double directionEncoderValue;
//    private double directionPower;
//
//    private double officialSetpoint = 0;
//
//    public SwerveDriveWheel(double p, double i, double d, DcMotorEx speedMotor, CRServo directionServo) {
//        directionController = new PIDController(p, i, d, 0);
//        this.speedMotor = speedMotor;
//        this.directionServo = directionServo;
//    }
//
//    /**
//     * Method will be called once to set a direction
//     * @param setpoint from 0 to 360 degrees angle? or -180 to 180 idk
//     */
//    public void setDirection(double setpoint) {
//
////        directionEncoderValue = BULK READ VARIABLE
//        // Convert encoder value to angle
//        // 2048 ticks
//        double currentAngle = directionEncoderValue; // NEED TO CONVERT THIS TO ANGLES LATER
//
//        // find closest angle to setpoint
//        double setpointAngle = closestAngle(currentAngle, setpoint);
//        // find closest angle to setpoint + 180
//        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
//        // if the closest angle to setpoint is shorter
//        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
//        {
//            // unflip the motor direction use the setpoint
//            directionServo.setDirection(DcMotor.Direction.FORWARD);
//            officialSetpoint = currentAngle + setpointAngle;
//        }
//        // if the closest angle to setpoint + 180 is shorter
//        else
//        {
//            // flip the motor direction and use the setpoint + 180
//            directionServo.setDirection(DcMotor.Direction.REVERSE);
//            officialSetpoint = currentAngle + setpointAngleFlipped;
//        }
//    }
//
//    /**
//     * Run everytime in the main loop to power the servo
//     */
//    public void loop() {
//        directionPower = directionController.calculatePID(officialSetpoint, directionEncoderValue);
//        directionServo.setPower(directionPower);
//    }
//
//    /**
//     * Get the closest angle between the given angles.
//     */
//    private static double closestAngle(double a, double b)
//    {
//        // get direction
//        double dir = (b % 360.0) - (a % 360.0);
//
//        // convert from -360 to 360 to -180 to 180
//        if (Math.abs(dir) > 180.0)
//        {
//            dir = -(Math.signum(dir) * 360.0) + dir;
//        }
//        return dir;
//    }
//
//}
