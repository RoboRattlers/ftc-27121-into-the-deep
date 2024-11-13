package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import java.util.SortedSet;

@TeleOp
public class EncoderReader extends OpMode {

    List<DcMotor> motors;
    SortedSet<String> motorNames;

    @Override
    public void init() {
        motors = hardwareMap.getAll(DcMotor.class);
        SortedSet<String> motorNames = hardwareMap.getAllNames(DcMotor.class);
    }

    @Override
    public void loop() {

        for (int i = 0; i < motors.size(); i++) {
            telemetry.addData("Motor", motors.get(i).getCurrentPosition());
        }

    }

}
