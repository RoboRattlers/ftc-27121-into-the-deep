package org.firstinspires.ftc.teamcode;

public class PIDController {

    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double setPoint = 0.0;
    private double output = 0.0;

    public PIDController(double kP, double kI, double kD) { // constructor
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getOutput() {
        return output;
    }

    public void update(double state) {

        double error = setPoint - state;
        //TODO: add integral and derivative terms
        output = error * kP;

    }

}
