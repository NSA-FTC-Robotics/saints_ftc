package org.firstinspires.ftc.teamcode;
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

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@SuppressWarnings("SingleStatementInBlock")
@Autonomous(name="Autonomous (B Crater)", group ="Concept")
//@Disabled
public class Auto_3 extends LinearOpMode {

   // HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor liftMotor;
    private DcMotor liftingArm;
    private DcMotor spinMotor;
    private Servo servo5;
    private Servo servo6;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    //Use values to make quick adjustments to code
    //Time is measured in milliseconds
    private int leftTraversalTime = 720;
    private int centerTraversalTime = 720;
    private int rightTraversalTime = 1100;
    private int leftTurnTime = 400;
    private int centerTurnTime = 400;
    private int rightTurnTime = 700;

    public void dontMove(int time){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(time);
    }
    public void dontMove(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
    // int time is in ms for all of the following methods
    // pow is the percent power
    private void strafeLeft(int time, double pow){
        frontLeft.setPower(-1 * pow);
        backLeft.setPower(-1 * pow);
        frontRight.setPower(-1 * pow);
        backRight.setPower(-1 * pow);
        sleep(time);
        }

        private void strafeRight(int time, double pow){
            frontLeft.setPower(1 * pow);
            backLeft.setPower(1 * pow);
            frontRight.setPower(1 * pow);
            backRight.setPower(1 * pow);
            sleep(time);
        }

        private void Forward(int time,double pow){
            frontLeft.setPower(1 * pow);
            backLeft.setPower(-1 * pow);
            frontRight.setPower(1 * pow);
            backRight.setPower(-1 * pow);
            sleep(time);
        }

        private void Backward(int time,double pow){
            frontLeft.setPower(-1 * pow);
            backLeft.setPower(1 * pow);
            frontRight.setPower(-1 * pow);
            backRight.setPower(1 * pow);
            sleep(time);
        }
        /* When using rightAngleTurn write the parameter as either "L" or "R"
        you can guess what that means. */
        private void rightAngleTurn(String direction){
          if(direction.startsWith("R")){
            frontLeft.setPower(1); //reverse this if doesnt work
            backLeft.setPower(-1);
            frontRight.setPower(-1);
            backRight.setPower(1);
            sleep(450);
          } else if(direction.startsWith("L")){
              frontLeft.setPower(-1);
              backLeft.setPower(1);
              frontRight.setPower(1);
              backRight.setPower(-1);
              sleep(800);
          } else telemetry.addLine("lmao who wrote this it doesn't work");
        }

        private void moveArm(double pow){
            liftMotor.setPower(pow * 0.5);
            sleep(1000);
            liftMotor.setPower(0);
        }

        private void spinSpinner(int time, double pow){
            spinMotor.setPower(pow);
            sleep(time);
            spinMotor.setPower(0);
        }

        private void findYellow(){
            boolean yellowFound = false;
            while (yellowFound) { //While we have NOT detected the color yellow
                if (sensorColor.red() < 16 && sensorColor.red() > 12
                        && sensorColor.green() < 12 && sensorColor.green() > 8
                        && sensorColor.blue() < 8 && sensorColor.blue() > 4)  //Checks the center
                {
                    telemetry.addData("Garamond","Yellow Found");
                    yellowFound = true;
                }
                else {
                    strafeRight(700, 0.5);

                    if (sensorColor.red() < 16 && sensorColor.red() > 12
                            && sensorColor.green() < 12 && sensorColor.green() > 8
                            && sensorColor.blue() < 8 && sensorColor.blue() > 4){ //Checks the
                        telemetry.addData("Garamond","Yellow Found");
                        yellowFound = true;
                    }
                    else{
                        strafeLeft(1400, 0.5);

                        if (sensorColor.red() < 16 && sensorColor.red() > 12
                                && sensorColor.green() < 12 && sensorColor.green() > 8
                                && sensorColor.blue() < 8 && sensorColor.blue() > 4){ //Checks the
                            telemetry.addData("Garamond","Yellow Found");
                            yellowFound = true;
                        }
                    }
                }
            }
            Backward(750,0.5);
            dontMove(1000);
            Forward(750, 1);
            }


    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        liftMotor = hardwareMap.get(DcMotor.class, "winch_motor");
        liftingArm = hardwareMap.get(DcMotor.class, "lifter_motor");
        spinMotor = hardwareMap.get(DcMotor.class, "spin_motor");
        servo5 = hardwareMap.get(Servo.class, "servo5");
        servo6 = hardwareMap.get(Servo.class, "servo6");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Garamond", "Resetting Encoders");
        telemetry.update();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftingArm.setDirection(DcMotor.Direction.FORWARD);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
       // encoderDrive(DRIVE_SPEED, 10, 10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
       // encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -10, -10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Garamond", "Press that PLAY button");
        telemetry.update();
        waitForStart();
      /* strafeLeft(1000,0.75);
        rightAngleTurn("L");
        dontMove();
        findYellow();*/
       // moveArm(0.5);
        liftingArm.setPower(-1);
        sleep(3200);
        liftingArm.setPower(0);

        Backward(250, 0.5);
        strafeLeft( 1000,0.75);
        rightAngleTurn("L");
        Forward(1000, 0.75);
        //spinSpinner(1000, -1);
        /*dontMove();
        moveArm(0.5);
        dontMove();
        moveArm(-0.5);
        Backward(1000, 1);
        dontMove(300);
       /* rightAngleTurn("L");
        Forward(3000,1);
        dontMove();
        moveArm(0.5);
*/


        telemetry.addData("Garamond", "Good luck Drivers.");
        telemetry.update();
        }
}