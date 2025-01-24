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

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Utils.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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


@TeleOp(name="iterativeTEST CURRENT")

public class iterativeTestnoSlides extends OpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;
    private DcMotor slide1 = null;
    private DcMotor slide2 = null;
//    private DcMotor reach = null;

    private CRServo intake2 = null; //port 0 exp hub.
    private Servo shoulder1 = null; //port 1 exp hub. right
    private Servo wrist1 = null; //not being used port 2
    private Servo intup1 = null; //port 3 expansion hub. left
    private Servo claw = null; // port 5 exp

    private Servo reach = null; //control 0 control hub
    private Servo intup2 = null; //port 1 control hub. right
    private Servo shoulder2 = null; //port 3 control hub. left
    private CRServo intake1 = null; //port 4 control hub.
    private Servo wrist2 = null; //port 5 control hub
    ;

    NormalizedColorSensor colorSensor;

    IMU imu;


    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during tahe robot configuration step on the DS or RC devices.
        lf = hardwareMap.get(DcMotor.class, "leftFront");
        lb = hardwareMap.get(DcMotor.class, "leftBack");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        rb = hardwareMap.get(DcMotor.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        //dave = hardwareMap.get(DcMotor.class, "dave");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        shoulder1 = hardwareMap.get(Servo.class, "shoulder1");
        shoulder2 = hardwareMap.get(Servo.class, "shoulder2");

        claw = hardwareMap.get(Servo.class, "claw");

        intup1 = hardwareMap.get(Servo.class, "intup1");
        intup2 = hardwareMap.get(Servo.class, "intup2");
        reach = hardwareMap.get(Servo.class, "reach");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        slide1.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist1.setPosition(WRIST1_UP);
        wrist2.setPosition(WRIST2_UP);

        intake1.setPower(0);
        intake2.setPower(0);

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

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        boolean intake = false;

        final double JOYSTICK_SEN = .007;

        double lx = Math.abs(gamepad1.left_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.left_stick_x;
        //lx is turning
        double rx = Math.abs(gamepad1.right_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_x;
        //rx is strafing
        double ry = Math.abs(gamepad1.right_stick_y)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_y;

        leftBackPower    = Range.clip(-lx - rx - ry, -1.0, 1.0);
        leftFrontPower    = Range.clip(-lx + rx - ry, -1.0, 1.0);
        rightFrontPower   = Range.clip(-lx + rx + ry, -1.0, 1.0);
        rightBackPower   = Range.clip(-lx - rx + ry, -1.0, 1.0) ;

        if (gamepad1.dpad_down) {
            setMotorPower(lf,lb,rf,rb,-.3,-.3,.3,.3);
        }
        if (gamepad1.dpad_up) {
            setMotorPower(lf,lb,rf,rb,.3,.3,-.3,-.3);
        }
        if (gamepad1.dpad_right) {
            setMotorPower(lf,lb,rf,rb,-.3,.3,-.3,.3);
        }
        if (gamepad1.dpad_left) {
            setMotorPower(lf,lb,rf,rb,.3,-.3,.3,-.3);
        }

        slide1.setPower(gamepad2.left_stick_y * SLIDES_MAX);
        slide2.setPower(gamepad2.left_stick_y * SLIDES_MAX);

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        colorSensor.setGain(2);

        boolean in = false;
        if (gamepad2.b) {
            //outtake
            intake = true;
            in = false;
            intake2.setPower(INTAKE_PWR);
            intake1.setPower(-INTAKE_PWR);
        }
        if (gamepad2.a) {
            //intake
            if (!intake) {
                in = true;
                intake1.setPower(INTAKE_PWR);
                intake2.setPower(-INTAKE_PWR);
                intake = true;
            } else {
                intake = false;
            }
            if (colors.blue > 0.5) {
                intake = true;
                in = false;
                intake2.setPower(INTAKE_PWR);
                intake1.setPower(-INTAKE_PWR);
            }
        }

        if(intake && in){
            intake1.setPower(INTAKE_PWR);
            intake2.setPower(-INTAKE_PWR);
        }
        if (gamepad2.right_bumper) {
            //wrist down
            wrist1.setPosition(WRIST1_DOWN);
            wrist2.setPosition(WRIST2_DOWN);

        }

        if (gamepad2.left_bumper) {
            //wrist moving up, intake goes on to keep block in
//            intake = true;
            intake1.setPower(INTAKE_PWR);
            intake2.setPower(-INTAKE_PWR);
            wrist1.setPosition(WRIST1_UP);
            wrist2.setPosition(WRIST2_UP);
        }

        if (gamepad2.x) {
            //intup up
            intup1.setPosition(INT_UP1);
            intup2.setPosition(INT_UP);
        }
        if (gamepad2.y) {
            //intup down
            intup1.setPosition(INT_DOWN1);
            intup2.setPosition(INT_DOWN);
        }

        if (!intake){
            intake1.setPower(0);
            intake2.setPower(0);
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

        if (gamepad1.a){
            reach.setPosition(SLIDES_IN);
        }
        if (gamepad1.b){
            reach.setPosition(SLIDES_OUT);
        }

        if (gamepad1.y){
            claw.setPosition(CLAW_IN);
        }
        if (gamepad1.x){
            claw.setPosition(CLAW_OUT);
        }

        double maxSpeed = .9;

        setMotorPower(lf,lb,rf,rb,leftFrontPower * maxSpeed,leftBackPower * maxSpeed,
                rightFrontPower * maxSpeed,rightBackPower * maxSpeed);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Quantum on top", intup1.getPosition());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.update();
    }
}

