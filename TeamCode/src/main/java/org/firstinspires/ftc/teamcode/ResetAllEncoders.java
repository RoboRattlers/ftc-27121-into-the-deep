package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import java.util.SortedSet;

@TeleOp
public class ResetAllEncoders extends LinearOpMode {

    @Override
    public void runOpMode() {

        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        motors.forEach((motor) -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });

    }

}
