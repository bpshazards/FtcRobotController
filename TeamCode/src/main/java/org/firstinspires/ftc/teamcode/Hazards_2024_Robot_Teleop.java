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

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This file contains an example of a Linear "OpMode".
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
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="2024_Hazard", group="Linear OpMode")
//@Disabled
public class Hazards_2024_Robot_Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private double MINDRIVE = 0.3;
    private DcMotor Linear_up = null;
    private DcMotor Linear_Forward = null;
    private CRServo intake_spinning =null;
    private Servo servo_arm_1;
    private CRServo Servo_bucket;
    private double Servo_bucket_pos = -0.8;
    private boolean intake_pos_toggle = true;
    private double intake_pos_low = 0.15;
    private double intake_pos_high = 0.9;
    private DistanceSensor Linear_Up_Dist = null;
    private ColorSensor Linear_Fwd_Color = null;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "drivelf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "drivelb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driverf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driverb");
        intake_spinning = hardwareMap.get(CRServo.class, "intake_spinning");
        servo_arm_1 = hardwareMap.get(Servo.class, "servoarm1");
        Linear_up = hardwareMap.get(DcMotor.class, "Linear_up");
        Servo_bucket = hardwareMap.get(CRServo.class, "SBucket");
        Linear_Forward = hardwareMap.get(DcMotor.class, "Linear_FW");
        Linear_Up_Dist = hardwareMap.get(DistanceSensor.class, "Linear_Up_Dist");
        Linear_Fwd_Color = hardwareMap.get(ColorSensor.class,"Linear_Fwd_Color");



        Linear_Forward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Linear_Forward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linear_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Linear_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        Linear_up.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Servo_bucket.setPower(Servo_bucket_pos);
        servo_arm_1.setPosition(intake_pos_high);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            leftFrontPower *= MINDRIVE + (1-MINDRIVE)*gamepad1.right_trigger;
            rightFrontPower *= MINDRIVE + (1-MINDRIVE)*gamepad1.right_trigger;
            leftBackPower *= MINDRIVE + (1-MINDRIVE)*gamepad1.right_trigger;
            rightBackPower *= MINDRIVE + (1-MINDRIVE)*gamepad1.right_trigger;

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            if (gamepad2.left_bumper) {
                intake_spinning.setPower(1);
            } else if (gamepad2.right_bumper) {
                intake_spinning.setPower(-1);
            } else {
                intake_spinning.setPower(0);
            }

            if (gamepad2.x) {
                servo_arm_1.setPosition(intake_pos_low);
            } else {
                servo_arm_1.setPosition(intake_pos_high);
            }

            telemetry.addData("Intake_pos_Toggle" , intake_pos_toggle);
            telemetry.addData( "Intake_pos",servo_arm_1.getPosition());



            if (gamepad2.dpad_left && (Linear_Forward.getCurrentPosition()<-1 || Linear_Fwd_Color.green()<800)) {
                Linear_Forward.setPower(.75);
            }else if (gamepad2.dpad_right && Linear_Forward.getCurrentPosition()>-2034) {
                Linear_Forward.setPower(-.75);
            }else {
                Linear_Forward.setPower(0);
            }

            if (Linear_Up_Dist.getDistance(DistanceUnit.CM)<10.0  && Linear_up.getCurrentPosition()<-100) {
                Linear_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Linear_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (Linear_Fwd_Color.green()>1200 && Linear_Forward.getCurrentPosition()>100){
                Linear_Forward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Linear_Forward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }



            // Start
            if (Linear_Forward.getCurrentPosition()<-750) {
                if (gamepad2.a) {
                    Servo_bucket.setPower(0.6);
                } else if (gamepad2.b) {
                    Servo_bucket.setPower(-0.8);
                }
                if (gamepad2.dpad_up && Linear_up.getCurrentPosition()<16400) {
                    Linear_up.setPower(1);
                } else if (gamepad2.dpad_down && (Linear_up.getCurrentPosition()>1  || Linear_Up_Dist.getDistance(DistanceUnit.CM)>10.0)) {
                    Linear_up.setPower(-1);
                }
            }

            if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                Linear_up.setPower(0);
            }

            telemetry.addData( "DpadUp:", gamepad2.dpad_up);
            telemetry.addData( "DpadDown:", gamepad2.dpad_down);
            telemetry.addData( "DpadLeft:", gamepad2.dpad_left);
            telemetry.addData( "DpadRight:", gamepad2.dpad_right);
            telemetry.addData("Linear Out Color.","red:%d,blue:%d,green:%d", Linear_Fwd_Color.red(),Linear_Fwd_Color.blue(),Linear_Fwd_Color.green());
            telemetry.addData("Linear_Up_Dist (cm)",Linear_Up_Dist.getDistance(DistanceUnit.CM));
            telemetry.addData("ServoBucket:",Servo_bucket.getPower());



            telemetry.addData("Status", "Run Time: " + runtime);
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("Gamepad Y", "%b", gamepad1.y);
            //telemetry.addData("Gamepad X", "%b", gamepad1.x);


            telemetry.addData("Forward encoder value", Linear_Forward.getCurrentPosition());
            telemetry.addData("Up encoder value", Linear_up.getCurrentPosition());

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}
