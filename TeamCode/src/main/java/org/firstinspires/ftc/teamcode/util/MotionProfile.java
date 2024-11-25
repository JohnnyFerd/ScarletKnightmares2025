package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotionProfile {
    private double startingTime = 0;
    private double acceleration_dt = 0, distance, halfway_distance, max_acceleration,
            max_velocity, acceleration_distance, deceleration_dt = 0,
            cruise_distance, cruise_dt = 0, deceleration_time, entire_dt = 0,
            start, end;
    private double max_deceleration, deceleration_distance, goal_velocity;
    private Telemetry telemetry;

    private double instantPos = 0;
    public double distanceTraveled = 0;
    private double instantVel = 0;
    private double instantAcl = 0;
    private boolean isBusy = false;
    private double timeElapsed = 0;

    private MotionProfileParameters parameters;
    private boolean isBackwards = false;
    private ElapsedTime timer;

    public MotionProfile(Telemetry telemetry) {
        this.telemetry = telemetry;
        timer = new ElapsedTime();
    }

    public double getEntireMPTime() {
        return entire_dt;
    }
    public double getTimeElapsed() {
        return timeElapsed;
    }

    public void addTelemetry() {
        telemetry.addData("    Acceleration dt", acceleration_dt);
        telemetry.addData("    Cruise dt", cruise_dt);
        telemetry.addData("    Deceleration dt", deceleration_dt);
        telemetry.addData("    Entire dt", entire_dt);
        telemetry.addData("    Goal Velocity", goal_velocity);
        telemetry.addData("    Goal Velocity", goal_velocity);
        telemetry.addData("    Distance", distance);
        telemetry.addData("    Time Elapsed", timeElapsed);
    }
    
    public void setProfile(MotionProfileParameters parameters) {
        isBusy = true;
        startingTime = timer.seconds();
        this.parameters = parameters;
        this.start = parameters.getStart();
        this.end = parameters.getEnd();
        this.max_acceleration = parameters.getMaxAcceleration();
        this.max_velocity = parameters.getMaxVelocity();
        this.max_deceleration = parameters.getMaxDeceleration();
        isBackwards = false;
        distance = end - start;

        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            isBusy = false;
            instantPos = end;
            distanceTraveled = 0;
            instantVel = 0;
            instantAcl = 0;
            return;
        }
        if (distance < 0) {
            max_velocity *= -1;
            max_acceleration *= -1;
            max_deceleration *= -1;
            isBackwards = true;
        }

        if (parameters.isAsymmetric()) {
            setAsymmetricProfile();
        }else {
            setSymmetricProfile();
        }
    }

    public void setSymmetricProfile() {
        // Calculate the time it takes to accelerate to max velocity
        acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (Math.abs(acceleration_distance) > Math.abs(halfway_distance)) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / max_velocity;
        deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deceleration_dt;

        // for back and forth test opmode
//        profileTime = entire_dt;
    }
    public void setAsymmetricProfile() {
        acceleration_dt = max_velocity / max_acceleration;
        deceleration_dt = max_velocity / max_deceleration;

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
        deceleration_distance = 0.5 * max_deceleration * Math.pow(deceleration_dt, 2);
        cruise_distance = distance - acceleration_distance - deceleration_distance;
        cruise_dt = cruise_distance / max_velocity;

        if ((cruise_distance < 0 && !isBackwards) || (cruise_distance > 0 && isBackwards)) {
            cruise_distance = 0;
            cruise_dt = 0;
            goal_velocity = Math.sqrt( (distance * 2 * max_deceleration * max_acceleration) / (max_acceleration + max_deceleration) );
            if (isBackwards) {
                goal_velocity = -1.0 * Math.abs(goal_velocity);
            }
            acceleration_dt = goal_velocity / max_acceleration;
            deceleration_dt = goal_velocity / max_deceleration;

            max_velocity = goal_velocity;

            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            deceleration_distance = 0.5 * max_deceleration * Math.pow(deceleration_dt, 2);
        }

        entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        deceleration_time = acceleration_dt + cruise_dt;
    }

    public void updateState() {
        if (parameters.isAsymmetric()) {
            updateAsymmetricState();
        }else {
            updateSymmetricState();
        }
    }

    public void updateAsymmetricState() {
        isBusy = true;
        timeElapsed = timer.seconds() - startingTime;
        timeElapsed = Math.abs(timeElapsed);

        // if no motion profile is set
        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            isBusy = false;
            instantPos = end;
            distanceTraveled = 0;
            instantVel = 0;
            instantAcl = 0;
            return;
        }

        // if motion profile is done
        if (timeElapsed > entire_dt) {
            isBusy = false;
            instantPos = end;
            distanceTraveled = end - start;
            instantVel = 0;
            instantAcl = 0;
            return;
        }

        // if we're accelerating
        if (timeElapsed < acceleration_dt) {
            // use the kinematic equation for acceleration
            instantPos = 0.5 * max_acceleration * Math.pow(timeElapsed, 2);
            distanceTraveled = instantPos;
            instantPos += start;
            instantVel = max_acceleration * timeElapsed;
            instantAcl = max_acceleration;
            return;
        }

        // if we're cruising
        else if (timeElapsed < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = timeElapsed - acceleration_dt;

            // use the kinematic equation for constant velocity
            instantPos = acceleration_distance + max_velocity * cruise_current_dt;
            distanceTraveled = instantPos;
            instantPos += start;
            instantVel = max_velocity;
            instantAcl = 0;
            return;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            double decelerationTime = timeElapsed - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            instantPos = acceleration_distance + cruise_distance + max_velocity * decelerationTime - 0.5 * max_deceleration * Math.pow(decelerationTime, 2);
            distanceTraveled = instantPos;
            instantPos += start;
            instantVel = max_velocity - max_deceleration * (timeElapsed - deceleration_time);
            instantAcl = -1.0 * max_deceleration;
            return;
        }
    }

    public void updateSymmetricState() {
        isBusy = true;
        timeElapsed = timer.seconds() - startingTime;
        timeElapsed = Math.abs(timeElapsed);

        // if no motion profile is set
        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            isBusy = false;
            instantPos = end;
            distanceTraveled = 0;
            instantVel = 0;
            instantAcl = 0;
            return;
        }

        // if motion profile is done
        if (timeElapsed > entire_dt) {
            isBusy = false;
            instantPos = end;
            distanceTraveled = end - start;
            instantVel = 0;
            instantAcl = 0;
            return;
        }

        // if we're accelerating
        if (timeElapsed < acceleration_dt) {
            // use the kinematic equation for acceleration
            instantPos = 0.5 * max_acceleration * Math.pow(timeElapsed, 2);
            distanceTraveled = instantPos;
            instantPos += start;
            instantVel = max_acceleration * timeElapsed;
            instantAcl = max_acceleration;
            return;
        }

        // if we're cruising
        else if (timeElapsed < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = timeElapsed - acceleration_dt;

            // use the kinematic equation for constant velocity
            instantPos = acceleration_distance + max_velocity * cruise_current_dt;
            distanceTraveled = instantPos;
            instantPos += start;
            instantVel = max_velocity;
            instantAcl = 0;
            return;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            double decelerationTime = timeElapsed - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            instantPos = acceleration_distance + cruise_distance + max_velocity * decelerationTime - 0.5 * max_acceleration * Math.pow(decelerationTime, 2);
            distanceTraveled = instantPos;
            instantPos += start;
            instantVel = max_velocity - max_acceleration * (timeElapsed - deceleration_time);
            instantAcl = -1.0 * max_acceleration;
            return;
        }
    }
    public double getInstantPosition() {
        return instantPos;
    }
    public double getInstantVelocity() {
        return instantVel;
    }
    public double getInstantAcceleration() {
        return instantAcl;
    }
    public boolean isBusy() {
        return isBusy;
    }
}
