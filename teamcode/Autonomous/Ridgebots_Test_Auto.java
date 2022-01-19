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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Test", group="Iterative Opmode")
public class Ridgebots_Test_Auto extends OpMode
{
    // Declare OpMode members.
    // Ratio from alliance shipping hub to duck circle is 5:3(5 x direction, 3 y direction)
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor bl = null;
    private DcMotor fr = null;
    private DcMotor br = null;
    private DcMotor lift = null;
    private DcMotor spin = null;
    private DistanceSensor distanceSensor = null;

    private Servo claw = null;

    private boolean liftIsBusy = false;

    int stage = 0;
    boolean SPIN = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

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

        motorInit(fl, true, "without");
        motorInit(bl, true, "without");
        motorInit(fr, false, "without");
        motorInit(br, false, "without");
        motorInit(lift, true, "reset");
        motorInit(spin, true, "without");

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        lift.setTargetPosition(0);
        initGyro();
        stopMotors();
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
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setPosition(1.0d);
        lift.setPower(1);
        startMotors();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        switch (stage){
            case 0:
                runtime.startTime();
                startMotors();
                if (angles.firstAngle > -90.0){
                mecanumDrive_Cartesian(0.6d, 0.0d, 0.2d);}
                else {
                    stopMotors();
                    nextStage(0);}
                break;
            case 1:
                runtime.startTime();
                startMotors();
                if(angles.firstAngle > -180){
                    mecanumDrive_Cartesian(0.6d, 0.0d, -0.5d);
                } else {nextStage(0); }
                break;
            default:
                stopMotors();
                stop();
                break;
        }

        telemetry.addData("Stage: ", stage);
       //telemetry.addData("Position: ", lift.getCurrentPosition());
        telemetry.addData("bl: ", bl.getCurrentPosition());
        telemetry.addData("br: ", br.getCurrentPosition());
        telemetry.addData("range", String.format("%.01f mm", distanceSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", distanceSensor.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", distanceSensor.getDistance(DistanceUnit.INCH)));
       // telemetry.addData("Gyro: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        telemetry.addData("Gyro Angle: ", angles.firstAngle);
        telemetry.addData("Time: ", runtime.seconds());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        stopMotors();
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        } else if (encoder == "with"){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void stopMotors(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void startMotors(){
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void startMotorsMech(){
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void forward(double distance, boolean forward, double power){
        if(forward){
        if (br.getCurrentPosition() < distance){
            mecanumDrive_Cartesian(0.0d ,-power, 0.0d);
        } else {stopMotors(); stage++; } }
        else {
            if (br.getCurrentPosition() > -distance){
                mecanumDrive_Cartesian(0.0d ,power, 0.0d);
            } else {stopMotors(); runtime.reset(); stage++; }
        }
    }
    public void mecanum(double distance, boolean left, double power){
        if(left){
            if (br.getCurrentPosition() > -distance){
                mecanumDrive_Cartesian(-power, 0.0d, 0.0d);
            } else { stopMotors(); stage++; }
        } else { if (br.getCurrentPosition() < distance){
            mecanumDrive_Cartesian(power, 0.0d, 0.0d);
        } else { stopMotors(); runtime.reset(); stage++; } }
    }
    //ccw is positive
    public void tankTurn(double degree, boolean ccw, double power){
        if (ccw){
            if (angles.firstAngle < degree){
                fl.setPower(-power);
                bl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
            } else {stopMotors(); runtime.reset(); stage++;}
        } else {
            if (angles.firstAngle > degree){
                fl.setPower(power);
                bl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);
            } else {stopMotors(); runtime.reset(); stage++;}
        }
    }

    public void nextStage(int stages){
        if(stages == 0){
            stopMotors();
            stage++;
        } else { stopMotors(); stage = stages;}
    }

}
