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

// rev duo imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

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

@TeleOp(name="Mastadons Teleop", group="Iterative Opmode")
public class MastadonsTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /*
    * wheels
    */
    private DcMotor backLeftMotor   = null;
    private DcMotor backRightMotor  = null;
    private DcMotor frontLeftMotor  = null;
    private DcMotor frontRightMotor = null;
    private double y;
    private double x;
    private float rx;
    private double denominator;

    /*
    * lift
    */
    private DcMotor lift = null;
    private final int minLiftPosition = 0;
    private final int maxLiftPosition = 100;
    private int litPosition = 0;

    /*
    * arm
    */
    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private final int minArmPosition = 0;
    private final int maxArmPosition = 100;
    private final int armHomePosition = 20;
    private int armPosition = 0;

    /*
    * wrist (values MUST be between 0.0 and 1.00)
    */
    private Servo wrist = null;
    private final double wristHomePosition = 0.75;
    private double wristPosition = 0.50;

    /*
    * gripper (values MUST be between 0.0 and 1.00)
    */
    private Servo gripper = null;
    private final double gripperClosedPosition = 0.65;
    private final double gripperOpenPosition   = 0.20;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * setup driver's wheels
         */
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
         * setup driver's lift
         */
        lift = hardwareMap.get(DcMotor.class, "robotLiftMotor");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*
         * setup operator's arms
         */
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armLeft.setDirection(DcMotor.Direction.FORWARD);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setPower(0.0);

        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armRight.setDirection(DcMotor.Direction.REVERSE);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setPower(0.0);

        /*
         * setup operator's wrist
         */
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(wristHomePosition);

        /*
         * setup operator's gripper
         */
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(gripperOpenPosition);

        telemetry.addData("Mastadons Status", "Initialized");
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

        // Reset lift encoder
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset our wrist and gripper
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setTargetPosition(armHomePosition);
        armRight.setTargetPosition(armHomePosition);
        armLeft.setPower(1.0);
        armRight.setPower(1.0);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wrist.setPosition(wristHomePosition);
        gripper.setPosition(gripperOpenPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double armPower;
        double leftPower;
        double rightPower;
        double liftPower;

        /*
         * GAMEPAD 1: DRIVER CONTROLS -------------------------------
         */

        // drive forward/backwards + side-to-side
        y  = -gamepad1.left_stick_y * 0.8;
        x  = gamepad1.right_stick_x * 0.8;
        rx = gamepad1.left_stick_x  * 2;

        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

        backLeftMotor.setPower(((y - x) + rx) / denominator);
        backRightMotor.setPower(((y + x) - rx) / denominator);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        frontRightMotor.setPower(((y - x) - rx) / denominator);

        // lower lift
        if (gamepad1.left_bumper)
        {
            liftPosition -= 10;
        }

        // raise lift
        if (gamepad1.right_bumper)
        {
            liftPosition += 10;
        }

        // limit how far up or down we raise our lift
        liftPosition = Math.min(maxLiftPosition, Math.max(minLiftPosition, liftPosition));
        lift.setTargetPosition(liftPosition);

        // find out how power to send to our lift motor
        liftPower = 0.01 * (liftPosition - lift.getCurrentPosition());
        lift.setPower(liftPower);

        /*
         * GAMEPAD 2: OPERATOR CONTROLS -----------------------------
         */

        // lower arm
        if (gamepad2.left_trigger > 0.01)
        {
            armPosition -= 10;
        }

        // raise arm
        if (gamepad2.right_trigger > 0.01)
        {
            armPosition += 10;
        }

        // set our arm position
        armPosition = Math.min(maxArmPosition, Math.max(minArmPosition, armPosition));
        armLeft.setTargetPosition(armPosition);
        armRight.setTargetPosition(armPosition);
        armPower = 0.01 * (armPosition - armLeft.getCurrentPosition());
        armLeft.setPower(armPower);
        armRight.setPower(armPower);

        // lower wrist
        if (gamepad2.left_bumper)
        {
            wristPosition -= 0.01;
        }

        // raise wrist
        if (gamepad2.right_bumper)
        {
            wristPosition += 0.01;
        }

        // set our wrist position
        wristPosition = Math.min(1.0, Math.max(0.0, wristPosition));
        wrist.setPosition(wristPosition);

        // close gripper to pick up a pixel
        if (gamepad2.a)
        {
            gripper.setPosition(gripperOpenPosition);
        }
        // open gripper
        else
        {
            gripper.setPosition(gripperClosedPosition);
        }

        /*
         * TELEMETRY UPDATES ----------------------------------------
         */

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Arm Position", armLeft.getCurrentPosition());
        telemetry.addData("Arm Power", armLeft.getPower());

        telemetry.addData("Wrist Position", wristPosition);
        telemetry.addData("Wrist Power", wristPower);

        telemetry.addData("Lift Position", liftPosition);
        telemetry.addData("Lift Power", liftPower);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
        gripper.setPosition(gripperOpenPosition);
        wrist.setPosition(wristHomePosition);
    }

}