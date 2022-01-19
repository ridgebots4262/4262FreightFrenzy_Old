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

package org.firstinspires.ftc.teamcode.TeleOp;
import java.util.Scanner;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Teleop_Joystick", group = "Iterative Opmode")
public class Ridgebots_TeleOp_Joystick extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor bl = null;
    private DcMotor fr = null;
    private DcMotor br = null;


    private DcMotor lift = null;
    private DcMotor spin = null;

    private Servo claw = null;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private boolean liftIsBusy = false;

    boolean SPIN = false;

    double leftY = 0;

    double leftX = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");
        lift = hardwareMap.get(DcMotor.class, "lift");
        spin = hardwareMap.get(DcMotor.class, "spin");

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);

        motorInit(fl, true, "without");
        motorInit(bl, true, "without");
        motorInit(fr, false, "without");
        motorInit(br, false, "without");
        motorInit(lift, true, "reset");
        motorInit(spin, true, "without");

        lift.setTargetPosition(0);
        initGyro();
        // Tell the driver that initialization is complete.
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

        claw.setPosition(1.0d);
        lift.setTargetPosition(0);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double turn = gamepad1.right_stick_x;
        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;

        if (gamepad1.right_bumper) {
            mecanumDrive_Cartesian(leftX / 2, leftY / 2, turn / 2);
        } else if (gamepad1.left_bumper) {
            mecanumDrive_Cartesian(leftX / 4, leftY / 4, turn / 4);
        } else {
            mecanumDrive_Cartesian(leftX, leftY, turn);
        }

        if (gamepad2.dpad_down) {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(0);
            lift.setPower(1);
        } else if (gamepad2.dpad_up) {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(4915);
            lift.setPower(1);
        }
        if (gamepad2.right_bumper){
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(0.9d);
        }
        else if(gamepad2.left_bumper){
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(-0.9d);
        } else if(lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            lift.setPower(0.0d);
        }

        if (gamepad2.b) {
            spin.setPower(-0.3d);
        } else if (gamepad2.x) {
            spin.setPower(0.3d);
        } else {
            spin.setPower(0);
        }

        if (gamepad2.right_trigger >= 0.3d){ claw.setPosition(1.0d); }
        else if(gamepad2.left_trigger >= 0.3d) { claw.setPosition(0.2d); }

        telemetry.addData("Position: ", lift.getCurrentPosition());
        telemetry.addData("BL: ", bl.getCurrentPosition());
        telemetry.addData("BR: ", br.getCurrentPosition());
        telemetry.addData("Gyro: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        telemetry.addData("Gyro Angle: ", angles.firstAngle);
        telemetry.addData("Lift Mode: ", lift.getMode());
        telemetry.addData("Target Position: ", lift.getTargetPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void motorInit(DcMotor motor, boolean reverse, String encoder) {
        if (reverse) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        if (encoder == "without") {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (encoder == "reset") {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void mecanumDrive_Cartesian(double x, double y, double rightx) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        final double v1 = r * Math.cos(robotAngle) + (rightx / 1.5);
        final double v2 = r * Math.sin(robotAngle) - (rightx / 1.5);
        final double v3 = r * Math.sin(robotAngle) + (rightx / 1.5);
        final double v4 = r * Math.cos(robotAngle) - (rightx / 1.5);

        fl.setPower(v1);
        fr.setPower(v2);
        bl.setPower(v3);
        br.setPower(v4);
    }

    public void initGyro(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

}