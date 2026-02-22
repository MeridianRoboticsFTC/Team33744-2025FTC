package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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


/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Basic Far Shooting Auto", group="Robot")

public class BlueFarShootingAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor flywheelR = null;
    private DcMotor flywheelL = null;

    private Servo rightIntake = null;

    private Servo leftIntake = null;

    private IMU imu  = null;

    static final double     FORWARD_SPEED = 0.5;
    static final double     TURN_SPEED    = 0.3;

    static final double     FORWARD_SHOOTING_SPEED = 0.25;


    // @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )

                )

        );

        imu.resetYaw();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        flywheelL = hardwareMap.get(DcMotor.class, "flywheelL");
        flywheelR = hardwareMap.get(DcMotor.class, "flywheelR");
        leftIntake = hardwareMap.get(Servo.class, "left_intake");
        rightIntake = hardwareMap.get(Servo.class, "right_intake");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        leftIntake.setDirection(Servo.Direction.FORWARD);
        flywheelR.setDirection(DcMotor.Direction.REVERSE);
        flywheelL.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        flywheelL.setPower(0.7);
        flywheelR.setPower(0.7);

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward for 3 seconds

        backLeftDrive.setPower(FORWARD_SPEED);
        backRightDrive.setPower(FORWARD_SPEED);
        frontLeftDrive.setPower(FORWARD_SPEED);
        frontRightDrive.setPower(FORWARD_SPEED);
        sleep(250);

        frontLeftDrive.setPower(-TURN_SPEED);
        backLeftDrive.setPower(-TURN_SPEED);
        frontRightDrive.setPower(TURN_SPEED);
        backRightDrive.setPower(TURN_SPEED);
        sleep(250);

        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        sleep(4000);

        rightIntake.setPosition(0.75);
        leftIntake.setPosition(0.75);
        sleep(500);
        rightIntake.setPosition(0.0);
        leftIntake.setPosition(0.0);
        sleep(4000);
        rightIntake.setPosition(0.75);
        leftIntake.setPosition(0.75);
        sleep(500);
        rightIntake.setPosition(0.0);
        leftIntake.setPosition(0.0);
        sleep(5000);
        rightIntake.setPosition(0.75);
        leftIntake.setPosition(0.75);
        sleep(500);
        rightIntake.setPosition(0.0);
        leftIntake.setPosition(0.0);
        sleep(3000);
        backLeftDrive.setPower(FORWARD_SPEED);
        backRightDrive.setPower(FORWARD_SPEED);
        frontRightDrive.setPower(FORWARD_SPEED);
        frontLeftDrive.setPower(FORWARD_SPEED);
        sleep(200);


        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
//        leftDrive.setPower(TURN_SPEED);
//        rightDrive.setPower(-TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3:  Drive Backward for 1 Second
//        leftDrive.setPower(-FORWARD_SPEED);
//        rightDrive.setPower(-FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }

        // Step 4:  Stop
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
