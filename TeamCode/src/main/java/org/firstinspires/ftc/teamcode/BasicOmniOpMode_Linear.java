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

import android.text.style.SubscriptSpan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




/**
 * This function is executed when this OpMode is selected from the Driver Station.
 */

@TeleOp(name="Hazards_2024-2025_TeleOp", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    private static final double spinner_max = 0.5;
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Linear_up = null;
    private DcMotor linear_front = null;
   // private CRServo drone_servo;2
   // private DcMotor pixesl_extender = null;
    private Servo servo_grab;
  //  private Servo servo_grab2;
 //   private CRServo servo_rotate1;
  //  private CRServo servo_rotate2;
  //  private CRServo Suspensionservo;
  //  private DcMotor Suspensionmotor = null;


    boolean x_changed = false;
    boolean dpad_l2_changed = false;
    boolean dpad_r2_changed = false;




    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "drivelf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "drivelb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driverf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driverb");
        Linear_up =hardwareMap.get(DcMotor.class, "Slide_up");
        servo_grab = hardwareMap.get(Servo.class, "SGrab");
        linear_front = hardwareMap.get(DcMotor.class, "Slide_forward");
       /** drone_servo = hardwareMap.get(CRServo.class, "drone_servo");
        pixesl_extender = hardwareMap.get(DcMotor.class, "articulator");
        Suspensionservo = hardwareMap.get(CRServo.class, "Suspensionservo");
        Suspensionmotor = hardwareMap.get(DcMotor.class, "Suspensionmotor");;
        servo_grab2 = hardwareMap.get(Servo.class, "SGrab2");
        servo_rotate1 = hardwareMap.get(CRServo.class, "SRotate1");
        servo_rotate2 = hardwareMap.get(CRServo.class, "SRotate2");**/

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
        // LinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        linear_front.setDirection(DcMotorSimple.Direction.FORWARD);

        Linear_up.setDirection(DcMotor.Direction.REVERSE);
        //pixesl_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       //ixesl_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       //uspensionmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       //uspensionmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       //uspensionmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //xesl_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
            double min = 0.2;
            double throttle = min + (max - min) * gamepad1.right_trigger;
            double LFP = leftFrontPower * throttle;
            double RFP = rightFrontPower * throttle;
            double LBP = leftBackPower * throttle;
            double RBP = rightBackPower * throttle;
            leftBackDrive.setPower(LBP);
            leftFrontDrive.setPower(LFP);
            rightFrontDrive.setPower(RFP);
            rightBackDrive.setPower(RBP);
            //Spinner Example Code

            if (gamepad1.a) {
                Linear_up.setPower(0.6);
            } else {
                Linear_up.setPower(0);}
            if(gamepad1.b) {
                Linear_up.setPower(-0.6);
            }else {
                Linear_up.setPower(0);
            }
            // Start


            if (gamepad1.x) {
                linear_front.setPower(1);
            } else if (gamepad1.y) {
                linear_front.setPower(-1);
            }else {
                linear_front.setPower(0);}

            if (gamepad1.a) {
                linear_front.setPower(1);
            } else if (gamepad1.b) {
                linear_front.setPower(-1);
            }else {
                linear_front.setPower(0);}





            //End
          /*if (gamepad2.dpad_up) {
                pixesl_extender.setPower(1.0);
            } else if (gamepad2.dpad_down) {
                pixesl_extender.setPower(-1.0);
            } else {
                pixesl_extender.setPower(0);
            }
            if (gamepad2.x && !x_changed) {
                if (Suspensionservo.getPower() == -1.0)
                    Suspensionservo.setPower(1.0);
                else Suspensionservo.setPower(-1.0);
                x_changed = true;
            } else if (!gamepad2.x)
                x_changed = false;

            if (gamepad2.b) {
                Suspensionmotor.setPower(1);
            } else if (gamepad2.a) {
                Suspensionmotor.setPower(-1);
            } else {
                Suspensionmotor.setPower(0);
            */
           /* if (gamepad2.left_bumper && !dpad_l2_changed) {
                if (servo_grab.getPosition() == 0.1) servo_grab.setPosition(0.8);
                else servo_grab.setPosition(0.1);
                dpad_l2_changed = true;
            } else if (!gamepad2.left_bumper) dpad_l2_changed = false;

            if (gamepad2.right_bumper && !dpad_l2_changed) {
                if (servo_grab.getPosition() == 0.3) servo_grab.setPosition(0.6);
                else servo_grab.setPosition(0.3);
                dpad_l2_changed = true;
            } else if (!gamepad2.right_bumper) dpad_l2_changed = false;
               */

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
            //servo_grab.
            if (gamepad2.dpad_up) {
                servo_grab.setPosition(0);
            }else if(gamepad2.dpad_down) {
                servo_grab.setPosition(1);
            }else {
                servo_grab.setPosition(.5);
            }

            telemetry.update();

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Gamepad A", "%b", gamepad1.a);
            telemetry.update();

           /*f (gamepad1.a) {
                drone_servo.setPower(1);
            } else {
                drone_servo.setPower(0);
            }*/
            // Put loop blocks here.

        }// while Opmode
    }//RunOpmode
}//LinearOpmode