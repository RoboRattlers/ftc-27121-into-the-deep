/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
//@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotorEx frDrive;
    private DcMotorEx brDrive;
    private DcMotorEx blDrive;
    private DcMotorEx flDrive;
    private DcMotorEx arm;
    private DcMotorEx linear;
    private final PIDController armPidController = new PIDController(0.002, 0.0, 0.0); // 0.002 power per tick
    private final PIDController linearPidController = new PIDController(2.5, 0, 0.0); // 2.5 power per one slide extension
    private final double MAX_LINEAR_SLIDE_ENCODER = 2200;

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        frDrive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        brDrive = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        blDrive = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        flDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        linear = hardwareMap.get(DcMotorEx.class, "linear");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        //linear.setDirection(DcMotor.Direction.REVERSE); No idea if I will need this, uncomment if neccesary

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            double fb = -gamepad1.left_stick_y;
            double lr = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            frDrive.setPower(fb - lr - turn);
            brDrive.setPower(fb + lr - turn);
            blDrive.setPower(fb - lr + turn);
            flDrive.setPower(fb + lr - turn);

            // ARM FUNCTIONALITY
            {
                if (gamepad1.dpad_up) {
                    armPidController.setPoint = -1000.0;
                } else if (gamepad1.dpad_down) {
                    armPidController.setPoint = 0.0;
                }

                armPidController.update(arm.getCurrentPosition()); // tell the PID controller the arm's current position
                double staticFrictionTerm = -Math.signum(armPidController.getOutput()) * 0.05;
                // set point in ticks, times 1 half-revolution per 1000 ticks, times pi radians per half-revolution
                double counteractGravityTerm = Math.cos((arm.getCurrentPosition() / 2000.0) * Math.PI) * 0.3;
                // negative because for some reason, reversing the motor direction also reverses the encoder
                arm.setPower(-armPidController.getOutput() + staticFrictionTerm + counteractGravityTerm);

                telemetry.addData("Arm PID Output", armPidController.getOutput());
                telemetry.addData("Arm PID Setpoint", armPidController.setPoint);
                telemetry.addData("Arm Position", arm.getCurrentPosition());
            }

            // LINEAR SLIDE FUNCTIONALITY
            {
                double percent = linear.getCurrentPosition() / MAX_LINEAR_SLIDE_ENCODER;
                telemetry.addData("linear slide position", percent);

                if (gamepad1.right_trigger > 0.2) {
                    linearPidController.setPoint = 1.0;
                } else if (gamepad1.left_trigger > 0.2){
                    linearPidController.setPoint = 0.0;
                }

                linearPidController.update(percent);
                linear.setPower(linearPidController.getOutput());
            }

            telemetry.update();
        }
    }
}
