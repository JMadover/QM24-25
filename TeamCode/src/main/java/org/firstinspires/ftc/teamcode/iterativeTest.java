/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 */


@TeleOp(name="Driver Control")

public class iterativeTest extends OpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;
    private DcMotor lift = null;
    private DcMotor reach = null;

    private CRServo intake = null; //port 5 control hub.
    private Servo wrist1 = null; //port 1 expansion hub. right
    private Servo wrist2 = null; //port 4 control hub. left
    private Servo shoulder1 = null; //port 2 control hub. right
    private Servo shoulder2 = null; //port 3 control hub. left

    static final double WRIST1_DOWN = 0.2;
    static final double WRIST1_UP = 0.65;

    static final double WRIST2_DOWN = 0.2;
    static final double WRIST2_UP = 0.65;

    static final double INTAKE_PWR = .8;

    static final double SHOULDER1_DOWN = 1;
    static final double SHOULDER1_UP = 0.35;

    static final double SHOULDER2_DOWN = 0.4;
    static final double SHOULDER2_UP = 0.95;

    IMU imu;


    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        lf = hardwareMap.get(DcMotor.class, "leftFront");
        lb = hardwareMap.get(DcMotor.class, "leftBack");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        rb = hardwareMap.get(DcMotor.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");
        lift = hardwareMap.get(DcMotor.class, "lift");
        reach = hardwareMap.get(DcMotor.class, "reach");
        //dave = hardwareMap.get(DcMotor.class, "dave");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        shoulder1 = hardwareMap.get(Servo.class, "shoulder1");
        shoulder2 = hardwareMap.get(Servo.class, "shoulder2");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        reach.setDirection(DcMotor.Direction.FORWARD);
        //dave.setDirection(DcMotor.Direction.FORWARD);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reach.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist1.setPosition(WRIST1_UP);
        wrist2.setPosition(WRIST2_UP);
        shoulder1.setPosition(SHOULDER1_DOWN);
        shoulder2.setPosition(SHOULDER2_DOWN);
        intake.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
    }

    @Override
    public void loop() {

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
//            YawPitchRollAngles robotOrientation;
//            robotOrientation = imu.getRobotYawPitchRollAngles();
//            double robotyaw = robotOrientation.getYaw(AngleUnit.DEGREES);
//            double axial = oglateral*Math.cos(robotyaw)-ogaxial*Math.sin(robotyaw);
//            double lateral = oglateral*Math.cos(robotyaw)+ogaxial*Math.sin(robotyaw);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Stay within range

        double leftFrontPower = Range.clip(axial + lateral + yaw, -1.0, 1.0);
        double rightFrontPower = Range.clip(axial - lateral - yaw, -1.0,1.0);
        double leftBackPower = Range.clip(axial - lateral + yaw, -1.0,1.0);
        double rightBackPower = Range.clip(axial + lateral - yaw, -1.0,1.0);

        // Send calculated power to wheels

        if (gamepad1.dpad_up) {
            leftFrontPower = 0.1;
            rightFrontPower = 0.1;
            leftBackPower = 0.1;
            rightBackPower = 0.1;
        }
        if (gamepad1.dpad_down) {
            leftFrontPower = -0.1;
            rightFrontPower = -0.1;
            leftBackPower = -0.1;
            rightBackPower = -0.1;
        }
        if (gamepad1.dpad_left) {
            leftFrontPower = -0.3;
            rightFrontPower = 0.3;
            leftBackPower = 0.3;
            rightBackPower = -0.3;
        }
        if (gamepad1.dpad_right) {
            leftFrontPower = 0.3;
            rightFrontPower = -0.3;
            leftBackPower = -0.3;
            rightBackPower = 0.3;
        }
        if (gamepad1.right_bumper) {
            leftFrontPower = 0.4;
            rightFrontPower = -0.4;
            leftBackPower = 0.4;
            rightBackPower = -0.4;
        }
        if (gamepad1.left_bumper) {
            leftFrontPower = -0.4;
            rightFrontPower = 0.4;
            leftBackPower = -0.4;
            rightBackPower = 0.4;
        }


        lift.setPower(gamepad2.left_stick_y * .5);
        reach.setPower(-gamepad2.right_stick_y * .8);


        if (gamepad2.a) {
            //intake
            intake.setPower(INTAKE_PWR);
        }
        if (gamepad2.b) {
            //outtake
            intake.setPower(-INTAKE_PWR);
        } else if (!gamepad2.left_bumper) { //so it stays going while rotating
            intake.setPower(0);
        }
        if (gamepad2.right_bumper) {
            //wrist up
            wrist1.setPosition(WRIST1_UP);
            wrist2.setPosition(WRIST2_UP);
            intake.setPower(INTAKE_PWR);
        }

        if (gamepad2.left_bumper) {
            //wrist down
            wrist1.setPosition(WRIST1_DOWN);
            wrist2.setPosition(WRIST2_DOWN);
        }

        if (gamepad2.right_trigger >= 0.5) {
            //shoulder down
            shoulder1.setPosition(SHOULDER1_DOWN);
            shoulder2.setPosition(SHOULDER2_DOWN);
        }

        if (gamepad2.left_trigger >= 0.5) {
            //shoulder up
            shoulder1.setPosition(SHOULDER1_UP);
            shoulder2.setPosition(SHOULDER2_UP);
        }

        double maxSpeed = .7;
        lf.setPower(leftFrontPower * maxSpeed);
        rf.setPower(rightFrontPower * maxSpeed);
        lb.setPower(leftBackPower * maxSpeed);
        rb.setPower(rightBackPower * maxSpeed);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Quantum on top", '#' + 1);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }
}

