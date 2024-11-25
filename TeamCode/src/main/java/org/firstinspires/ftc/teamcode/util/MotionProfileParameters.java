package org.firstinspires.ftc.teamcode.util;

public class MotionProfileParameters {
    private boolean isAsymmetric = false;
    private int start;
    private int end;
    private int maxAcceleration;
    private int maxVelocity;
    private int maxDeceleration;

    public MotionProfileParameters(int start, int end, int maxAcceleration, int maxVelocity) {
        this.start = start;
        this.end = end;
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        maxDeceleration = 0;
    }

    public MotionProfileParameters(int start, int end, int maxAcceleration, int maxVelocity, int maxDeceleration) {
        isAsymmetric = true;
        this.start = start;
        this.end = end;
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.maxDeceleration = maxDeceleration;
    }

    public boolean isAsymmetric() {
        return isAsymmetric;
    }
    public int getStart() {
        return start;
    }
    public int getEnd() {
        return end;
    }
    public int getMaxAcceleration() {
        return maxAcceleration;
    }
    public int getMaxVelocity() {
        return maxVelocity;
    }
    public int getMaxDeceleration() {
        return maxDeceleration;
    }
}
