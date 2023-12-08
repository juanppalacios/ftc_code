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


/**
 * todo: add limits to the lift
 * todo: fix the wrist code
 * todo: fix starting position, retract arm, wrist, and lift
 * todo: fix our auton code, may just use what we had before
 */

package org.firstinspires.ftc.teamcode;

// rev duo imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * ------------------------------------------------------------------
 * DRIVER GAMEPAD 1 MAP:
 *
 * - left bumper: lower lift
 * - right bumper: raise lift
 *
 * - left joystick: move forward/backward
 * - right joystick: move left/right
 *
 * ------------------------------------------------------------------
 * OPERATOR GAMEPAD 2 MAP:
 *
 * - left trigger: lower arm
 * - right trigger: raise arm
 *
 * - left bumper: lower wrist
 * - right bumper: raise wrist
 *
 * - button a: close gripper (while pressed)
 *
 * ------------------------------------------------------------------
 */

@TeleOp(name="Panthers Teleop", group="Iterative Opmode")
public class PanthersTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // driver mech-wheels
    private DcMotor backLeftMotor   = null;
    private DcMotor backRightMotor  = null;
    private DcMotor frontLeftMotor  = null;
    private DcMotor frontRightMotor = null;

    // driver lift
    private DcMotor robotLiftMotor = null;

    // operator motors
    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private Servo wrist = null;
    private Servo gripper = null;

    private boolean wheelsEnabled = true;
    private boolean armEnabled = true;
    private boolean pressed = true;

    private boolean manualMode = false;
    private double armSetpoint = 0.0;

    private final double armManualDeadband = 0.03;

    private final double gripperClosedPosition = 0.65;
    private final double gripperOpenPosition   = 0.20;

    private final double wristUpPosition   = 0.90;
    private final double wristDownPosition = 0.25;

    private final int armHomePosition = 5;
    private final int armIntakePosition = 45;
    private final int armScorePosition = 200;
    private final int armShutdownThreshold = 0

    private final int robotLiftHomePosition = -200;

    private int robotLiftPosition= 0;

    private final int liftMaximumHeight     =  100; // stops the lift from going too high
    private final int robotLiftUpPosition   =  200; // tell the motors to go to the highest point while we are pressing our buttons
    private final int robotLiftDownPosition = -200; // tell the motors to go to the lowest point while we are pressing our buttons
    private final int liftMinimumHeight     = -100; // stops the lift from going too low

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // enable our driver's mech-wheels
        if (wheelsEnabled){
            backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");
            backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");
            frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            robotLiftMotor  = hardwareMap.get(DcMotor.class, "robotLiftMotor");

            backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            robotLiftMotor.setDirection(DcMotor.Direction.FORWARD);

            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robotLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // enable our operator's arm
        if (armEnabled)
        {
            armLeft  = hardwareMap.get(DcMotor.class, "armLeft");
            armRight = hardwareMap.get(DcMotor.class, "armRight");
            gripper  = hardwareMap.get(Servo.class, "gripper");
            wrist    = hardwareMap.get(Servo.class, "wrist");

            armLeft.setDirection(DcMotor.Direction.FORWARD);
            armRight.setDirection(DcMotor.Direction.REVERSE);

            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armLeft.setPower(0.0);
            armRight.setPower(0.0);
        }

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        if (wheelsEnabled){

            // todo: test the homing position, may need to add a while loop here...?
            while (robotLiftMotor.getCurrentPosition() > liftMinimumHeight)
            {
                robotLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robotLiftMotor.setTargetPosition(robotLiftHomePosition);
                robotLiftMotor.setPower(1.0);
                robotLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            robotLiftMotor.setPower(0.0);
        }

        if (armEnabled)
        {
            // RESET OUR ARM
            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLeft.setTargetPosition(armHomePosition);
            armRight.setTargetPosition(armHomePosition);
            armLeft.setPower(1.0);
            armRight.setPower(1.0);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // RESET OUR WRIST AND GRIPPER
            gripper.setPosition(gripperOpenPosition);
            wrist.setPosition(wristUpPosition);
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double manualArmPower;
        double leftPower;
        double rightPower;
        double liftPower;

        if (wheelsEnabled)
        {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0);
            rightPower   = Range.clip(drive - turn, -1.0, 1.0);

            //DRIVE
            backLeftMotor.setPower(leftPower);
            backRightMotor.setPower(rightPower);
            frontLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);

            // driver pushes the y-button to lift up the lift with some maximum height
            if (gamepad1.right_bumper && (robotLiftPosition<= liftMaximumHeight)) {
                robotLiftPosition = robotLiftPosition + 1; // note: we can use this variable to track our max/min limits
                robotLiftMotor.setDirection(DcMotor.Direction.FORWARD);
                robotLiftMotor.setTargetPosition(robotLiftUpPosition);
                robotLiftMotor.setPower(1.0);
                robotLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // driver pushes the a-button to lower the lift with some minimum height
            if (gamepad1.left_bumper && (robotLiftPosition>= liftMinimumHeight)) {
                robotLiftPosition = robotLiftPosition - 1; // note: we can use this variable to track our max/min limits
                robotLiftMotor.setDirection(DcMotor.Direction.REVERSE);
                robotLiftMotor.setTargetPosition(robotLiftDownPosition);
                robotLiftMotor.setPower(-1.0);
                robotLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // otherwise, do not move the lift
            if (!gamepad1.y && !gamepad1.a) {
                robotLiftMotor.setPower(0.0);
            }

        }

        if (armEnabled)
        {

            if (gamepad2.start == pressed) // RE-ZERO ENCODER BUTTON
            {
                armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLeft.setPower(0.0);
                armRight.setPower(0.0);
            }


            if (gamepad2.left_trigger == pressed) // LOWER ARM
            {
                armLeft.setTargetPosition(armHomePosition);
                armRight.setTargetPosition(armHomePosition);
                armLeft.setPower(1.0);
                armRight.setPower(1.0);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.right_trigger == pressed) // RAISE ARM
            {
                armLeft.setTargetPosition(armScorePosition);
                armRight.setTargetPosition(armScorePosition);
                armLeft.setPower(1.0);
                armRight.setPower(1.0);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else // KEEP ARM WHERE IT IS
            {
                armLeft.setPower(0.0);
                armRight.setPower(0.0);
            }


            if (gamepad2.left_bumper == pressed) // LOWER WRIST
            {
                wrist.setPosition(wristDownPosition);
            }


            if (gamepad2.right_bumper == pressed) // RAISE WRIST
            {
                wrist.setPosition(wristUpPosition);
            }


            if (gamepad2.a == pressed) // CLOSE GRIPPER
            {
                gripper.setPosition(gripperOpenPosition);
            }
            else // OPEN GRIPPER
            {
                gripper.setPosition(gripperClosedPosition);
            }


        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Lift Motor Pos:", robotLiftMotor.getCurrentPosition());
        // telemetry.addData("Gamepad", "drive (%.2f), turn (%.2f)", drive, turn);
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        // telemetry.addData("Manual Power", manualArmPower);
        // telemetry.addData("Arm Pos:",
        //     "left = " +
        //     ((Integer)armLeft.getCurrentPosition()).toString() +
        //     ", right = " +
        //     ((Integer)armRight.getCurrentPosition()).toString());
        // telemetry.addData("Arm Pos:",
        //     "left = " +
        //     ((Integer)armLeft.getTargetPosition()).toString() +
        //     ", right = " +
        //     ((Integer)armRight.getTargetPosition()).toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){}

}
