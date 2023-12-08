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
 * todo: 
 * 
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
 * gamepad1 is the mech-wheel driver
 * gamepad2 is the arm operator
 */

@TeleOp(name="Panthers Auton", group="Iterative Opmode")
public class PanthersAuton extends OpMode
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

    private boolean manualMode = false;
    private double armSetpoint = 0.0;

    private final double armManualDeadband = 0.03;

    private final double gripperClosedPosition = 0.65;
    private final double gripperOpenPosition = 0.20;
    private final double wristUpPosition = 0.90;
    private final double wristDownPosition = 0.25;

    private final int armHomePosition = 5;
    private final int armIntakePosition = 45;
    private final int armScorePosition = 200;
    private final int armShutdownThreshold = 0;
    
    private final int LIFT_HOME_POSITION = 10;

    private int liftPosition = 0;
    private int gotoLiftPosition = 0;
    
    private final int LIFT_MAXIMUM_HEIGHT    = 20;    // stops the lift from going too high
    private final int robotLiftPosition_UP   = 100;
    private final int robotLiftPosition_DOWN = 30;
    private final int LIFT_MINIMUM_HEIGHT    = -100;    // stops the lift from going too low

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

            backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            
            robotLiftMotor = hardwareMap.get(DcMotor.class, "robotLiftMotor");
            robotLiftMotor.setDirection(DcMotor.Direction.FORWARD);
            robotLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // enable our operator's arm
        if (armEnabled)
        {
            armLeft  = hardwareMap.get(DcMotor.class, "armLeft");
            armRight = hardwareMap.get(DcMotor.class, "armRight");
            gripper = hardwareMap.get(Servo.class, "gripper");
            wrist = hardwareMap.get(Servo.class, "wrist");

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
            // robotLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // robotLiftMotor.setTargetPosition(LIFT_HOME_POSITION);
            // robotLiftMotor.setPower(1.0);
            // robotLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (armEnabled)
        {
            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLeft.setTargetPosition(armHomePosition);
            armRight.setTargetPosition(armHomePosition);
            armLeft.setPower(1.0);
            armRight.setPower(1.0);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            //DRIVE
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0);
            rightPower   = Range.clip(drive - turn, -1.0, 1.0);

            backLeftMotor.setPower(leftPower);
            backRightMotor.setPower(rightPower);
            frontLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);

            // driver pushes the y-button to lift up the lift with some maximum height
            if (gamepad1.y && (robotLiftMotor.getCurrentPosition() < LIFT_MAXIMUM_HEIGHT)) {
            // if (gamepad1.y) { // uncomment this for free running lift upwards
                gotoLiftPosition = gotoLiftPosition + 1;
                robotLiftMotor.setDirection(DcMotor.Direction.FORWARD);
                robotLiftMotor.setTargetPosition(robotLiftPosition_UP);
                robotLiftMotor.setPower(1.0);
                robotLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            
            // driver pushes the a-button to lower the lift with some minimum height
            if (gamepad1.a && (gotoLiftPosition > LIFT_MINIMUM_HEIGHT)) {
            // if (gamepad1.a){ // uncomment this for free running lift downwards
                gotoLiftPosition = gotoLiftPosition - 1;
                robotLiftMotor.setDirection(DcMotor.Direction.REVERSE);
                robotLiftMotor.setTargetPosition(robotLiftPosition_DOWN);
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
            //ARM & WRIST
            manualArmPower = gamepad2.right_trigger - gamepad2.left_trigger;
            if (Math.abs(manualArmPower) > armManualDeadband) {
                if (!manualMode) {
                    armLeft.setPower(0.0);
                    armRight.setPower(0.0);
                    armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    manualMode = true;
                }
                armLeft.setPower(manualArmPower);
                armRight.setPower(manualArmPower);
            }
            else {
                if (manualMode) {
                    armLeft.setTargetPosition(armLeft.getCurrentPosition());
                    armRight.setTargetPosition(armRight.getCurrentPosition());
                    armLeft.setPower(0.5);
                    armRight.setPower(0.5);
                    armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    manualMode = false;
                }

                //preset buttons
                if (gamepad2.a) {
                    armLeft.setTargetPosition(armHomePosition);
                    armRight.setTargetPosition(armHomePosition);
                    armLeft.setPower(0.5);
                    armRight.setPower(0.5);
                    armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(wristUpPosition);
                }
                else if (gamepad2.b) {
                    armLeft.setTargetPosition(armIntakePosition);
                    armRight.setTargetPosition(armIntakePosition);
                    armLeft.setPower(0.5);
                    armRight.setPower(0.5);
                    armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(wristDownPosition);
                }
                else if (gamepad2.y) {
                    armLeft.setTargetPosition(armScorePosition);
                    armRight.setTargetPosition(armScorePosition);
                    armLeft.setPower(1.0);
                    armRight.setPower(1.0);
                    armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(wristUpPosition);
                }
            }

            //Re-zero encoder button
            if (gamepad2.start) {
                armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLeft.setPower(0.0);
                armRight.setPower(0.0);
                manualMode = false;
            }

            //Watchdog to shut down motor once the arm reaches the home position
            if (!manualMode &&
            armLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
            armLeft.getTargetPosition() <= armShutdownThreshold &&
            armLeft.getCurrentPosition() <= armShutdownThreshold
            ) {
                armLeft.setPower(0.0);
                armRight.setPower(0.0);
                armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //GRIPPER
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                gripper.setPosition(gripperOpenPosition);
            }
            else {
                gripper.setPosition(gripperClosedPosition);
            }
        }

        liftPosition = robotLiftMotor.getCurrentPosition();
        telemetry.addData("Lift Pos:", liftPosition);
        telemetry.addData("Tracked Lift Pos:", gotoLiftPosition);
        // telemetry.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        // telemetry.addData("Gamepad", "drive (%.2f), turn (%.2f)", drive, turn);
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        // telemetry.addData("Manual Power", manualArmPower);
        telemetry.addData("Arm Pos:",
            "left = " +
            ((Integer)armLeft.getCurrentPosition()).toString() +
            ", right = " +
            ((Integer)armRight.getCurrentPosition()).toString());
        telemetry.addData("Arm Pos:",
            "left = " +
            ((Integer)armLeft.getTargetPosition()).toString() +
            ", right = " +
            ((Integer)armRight.getTargetPosition()).toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){}

}
