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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
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

enum DriveMode {
    DRIVE,
    DEPOSIT_BASKET,
    INTAKE,
    MANUAL_OVERRIDE
}

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
    private Servo wristRoll;
    private Servo wristPitch;
    private CRServo intake;

    private final PIDController armPidController = new PIDController(0.002, 0.0, 0.0); // 0.002 power per tick
    private final PIDController linearPidController = new PIDController(25, 0, 0.0); // 2.5 power per one slide extension
    private final double MAX_LINEAR_SLIDE_ENCODER = 2200;

    private DriveMode currentDriveMode = DriveMode.DRIVE;
    private double driveSpeedMult = 1.0;

    private Gamepad currentGamepad = new Gamepad();
    private Gamepad lastGamepad = new Gamepad();

    private double lastUpdateTime = 0.0;


    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        frDrive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        brDrive = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        blDrive = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        flDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        linear = hardwareMap.get(DcMotorEx.class, "linear");
        wristRoll = hardwareMap.get(Servo.class, "wristRoll");
        wristPitch = hardwareMap.get(Servo.class, "wristPitch");
        intake = hardwareMap.get(CRServo.class, "intake");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
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

            double runtime = getRuntime();
            double deltaTime = runtime - lastUpdateTime;
            lastUpdateTime = runtime;

            lastGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            double slidePosition = linear.getCurrentPosition() / MAX_LINEAR_SLIDE_ENCODER;

            switch (currentDriveMode) {
                case DRIVE: {

                    driveSpeedMult = 1.0;
                    linearPidController.setPoint = 0.0;
                    // only pivot down if slides are retracted; don't want to tip over
                    if (slidePosition < 0.3) {
                        armPidController.setPoint = 0.0;
                    }

                    wristRoll.setPosition(0.6);
                    wristPitch.setPosition(0.5);
                    intake.setPower( 0.0 );

                    if (currentGamepad.start && !lastGamepad.start) {
                        currentDriveMode = DriveMode.MANUAL_OVERRIDE;
                    } else if (currentGamepad.dpad_up && !lastGamepad.dpad_up) {
                        currentDriveMode = DriveMode.DEPOSIT_BASKET;
                    } else if (currentGamepad.dpad_down && !lastGamepad.dpad_down) {
                        currentDriveMode = DriveMode.INTAKE;
                    }

                    break;
                }
                case DEPOSIT_BASKET: {

                    driveSpeedMult = 0.5;
                    armPidController.setPoint = -1000.0;
                    boolean isArmInThreshold = Math.abs(arm.getCurrentPosition() - armPidController.setPoint) < 100.0;

                    // only extend if arm is pivoted up; once again, don't want to tip over
                    if (isArmInThreshold) {
                        linearPidController.setPoint = 1.0;
                    }

                    wristRoll.setPosition(0.5);
                    wristPitch.setPosition(0.8);

                    intake.setPower( -gamepad1.left_trigger );

                    if (currentGamepad.dpad_down && !lastGamepad.dpad_down) {
                        currentDriveMode = DriveMode.DRIVE;
                    }

                    break;
                }
                case INTAKE: {

                    driveSpeedMult = 0.5;
                    linearPidController.setPoint = 0.45;
                    armPidController.setPoint = 170;

                    // don't turn the wrist until extended a bit; it might catch on the chassis otherwise
                    if (slidePosition > 0.1) {
                        wristRoll.setPosition(0.5);
                        wristPitch.setPosition(0.6);
                    }

                    intake.setPower( gamepad1.right_trigger - gamepad1.left_trigger );

                    if (currentGamepad.dpad_up && !lastGamepad.dpad_up) {
                        currentDriveMode = DriveMode.DRIVE;
                    }

                    break;
                }
                case MANUAL_OVERRIDE: {

                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    double armMoveDir = (currentGamepad.right_bumper ? 1.0 : 0.0) - (currentGamepad.left_bumper ? 1.0 : 0.0);
                    arm.setPower( armMoveDir * 0.35 );
                    linear.setPower( (currentGamepad.right_trigger - currentGamepad.left_trigger) * 0.3 );

                    double wristRollMoveDir = (currentGamepad.dpad_right ? 1.0 : 0.0) - (currentGamepad.dpad_left ? 1.0 : 0.0);
                    wristRoll.setPosition(wristRoll.getPosition() + wristRollMoveDir * deltaTime * 4.0 );
                    double wristPitchMoveDir = (currentGamepad.dpad_up ? 1.0 : 0.0) - (currentGamepad.dpad_down ? 1.0 : 0.0);
                    wristPitch.setPosition(wristPitch.getPosition() + wristPitchMoveDir * deltaTime * 4.0 );

                    if (currentGamepad.x) {
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    if (currentGamepad.start && !lastGamepad.start) {
                        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        currentDriveMode = DriveMode.DRIVE;
                    }

                    break;
                }
            }

            // DRIVETRAIN FUNCTIONALITY
            {
                double fb = -currentGamepad.left_stick_y * driveSpeedMult;
                double lr = currentGamepad.left_stick_x * driveSpeedMult;
                double turn = -currentGamepad.right_stick_x * driveSpeedMult;
                frDrive.setPower(fb - lr - turn);
                brDrive.setPower(fb + lr - turn);
                blDrive.setPower(fb - lr + turn);
                flDrive.setPower(fb + lr + turn);
            }

            // ARM PID FUNCTIONALITY
            if (currentDriveMode != DriveMode.MANUAL_OVERRIDE) {

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

            // LINEAR SLIDE PID FUNCTIONALITY
            if (currentDriveMode != DriveMode.MANUAL_OVERRIDE) {
                telemetry.addData("linear slide position", slidePosition);

                linearPidController.update(slidePosition);
                linear.setPower(linearPidController.getOutput());
            }

            telemetry.update();
        }
    }
}
