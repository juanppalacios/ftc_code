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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name="Panthers Auton", group="Iterative Opmode")
public class PanthersAuton extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private double y;
    private double x;
    private float rx;
    private double denominator;

    private DcMotor lift = null;
    private final int minLiftPosition = 0;
    private final int maxLiftPosition = 100;
    private int litPosition = 0;

    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private final int minArmPosition = 0;
    private final int maxArmPosition = 100;
    private final int armHomePosition = 20;
    private int armPosition = 0;

    private Servo wrist = null;
    private final double wristHomePosition = 0.75;
    private double wristPosition = 0.50;

    private Servo gripper = null;
    private final double gripperClosedPosition = 0.65;
    private final double gripperOpenPosition = 0.20;

    @Override
    public void init() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lift = hardwareMap.get(DcMotor.class, "robotLiftMotor");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(wristHomePosition);

        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(gripperOpenPosition);

        telemetry.addData("Panthers Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    @Override
    public void loop() {
        double armPower;
        double leftPower;
        double rightPower;
        double liftPower;

        // ... (existing code)

        /*
         * AUTONOMOUS ACTIONS ---------------------------------------
         */

        // Move left a bit
        moveLeft();
        sleep(1000);  // Sleep for 1 second

        // Drive forward for three seconds
        driveForward();
        sleep(3000);  // Sleep for 3 seconds

        // Stop all motors
        stopMotors();

        // ... (existing code)

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arm Position", armLeft.getCurrentPosition());
        telemetry.addData("Arm Power", armLeft.getPower());
        telemetry.addData("Wrist Position", wristPosition);
        telemetry.addData("Wrist Power", wristPower);
        telemetry.addData("Lift Position", liftPosition);
        telemetry.addData("Lift Power", liftPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        gripper.setPosition(gripperOpenPosition);
        wrist.setPosition(wristHomePosition);
    }

    private void moveLeft() {
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(0.5);
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
    }

    private void driveForward() {
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
    }

    private void stopMotors() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }
}
