/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Bucket Side Auto", group = "Red")
public class BucketSide_Auto extends LinearOpMode {
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor Linear_up = null;
    private DcMotor Linear_Forward = null;
    private CRServo Servo_bucket;
    private Servo servo_arm_1;
    private double Servo_bucket_pos = -0.8;
    private double Servo_bucket_score = 0.6;
    private boolean intake_pos_toggle = true;
    private double intake_pos_low = 0.15;
    private double intake_pos_high = 0.9;
    private int Linear_up_pos = 15250;
    private DistanceSensor Linear_Up_Dist = null;
    private ColorSensor Linear_Fwd_Color = null;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive()) {
            Linear_up = hardwareMap.get(DcMotor.class, "Linear_up");
            Servo_bucket = hardwareMap.get(CRServo.class, "SBucket");
            servo_arm_1 = hardwareMap.get(Servo.class, "servoarm1");
            Linear_Up_Dist = hardwareMap.get(DistanceSensor.class, "Linear_Up_Dist");
            Linear_Forward = hardwareMap.get(DcMotor.class, "Linear_FW");
            Linear_Fwd_Color = hardwareMap.get(ColorSensor.class, "Linear_Fwd_Color");

            Linear_Forward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Linear_Forward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Linear_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Linear_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Linear_up.setDirection(DcMotor.Direction.REVERSE);

            Servo_bucket.setPower(Servo_bucket_pos);
            servo_arm_1.setPosition(intake_pos_high);

            // Initialize the robot hardware & Turn on telemetry
            robot.initialize(true);

            // Wait for driver to press start
            telemetry.addData(">", "Touch Play to run Auto");
            telemetry.update();

            waitForStart();
            robot.resetHeading();  // Reset heading to set a baseline for Auto

            // Run Auto if stop was not pressed.
                // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.
                servo_arm_1.setPosition(intake_pos_low);
                while (Linear_Forward.getCurrentPosition()>=-200) {
                    Linear_Forward.setTargetPosition(-200);
                    Linear_Forward.setPower(-1);
                    telemetry.addData("Current Pos-Set", Linear_Forward.getTargetPosition()-Linear_Forward.getCurrentPosition());
                    telemetry.update();
                }
                Linear_Forward.setPower(0);
                Linear_up.setTargetPosition(Linear_up_pos);
                Linear_up.setPower(1);
                robot.drive(5, 0.50, 0.2);// Push from wall 5 "
                robot.strafe(12, 0.50, 0.2); //Strafe infront of Triangle for bucket
                robot.turnTo(-45.0, 0.2, 0.2); // Rotate to line up with bucket
                while (Linear_up.getCurrentPosition()<Linear_up_pos) {
                    Linear_up.setPower(1);
                    telemetry.addData("Current Pos-Set", Linear_up.getTargetPosition()-Linear_up.getCurrentPosition());
                    telemetry.update();
                }
                Linear_up.setPower(0);
                Servo_bucket.setPower(Servo_bucket_score);
                sleep(1500);
                Servo_bucket.setPower(Servo_bucket_pos);
                robot.drive(20, 0.50, 0.15);
                Linear_up.setTargetPosition(100);
                Linear_up.setPower(-.5);
                robot.turnTo(0, 0.2, 0.15);
                robot.drive(36, 0.50, 0.15);
                robot.strafe(-14, 0.50, 0.15);
                while (Linear_Up_Dist.getDistance(DistanceUnit.CM)>10.0) {
                    Linear_up.setPower(-1);
                }
                Linear_up.setPower(0);
                servo_arm_1.setPosition(intake_pos_high);

            //Linear_Forward.setPower(0.75);

                /*
            if (Linear_Forward.getCurrentPosition()<-750) {
                Linear_Forward.setPower(0);
            }
                sleep(500);

            if (Linear_up.getCurrentPosition()<16400) {
                Linear_up.setPower(1);
            } else Linear_up.setPower(0);

                sleep(750);

                Servo_bucket.setPower(0.6);
                sleep(750);
                Servo_bucket.setPower(-0.8);
                sleep(750);

                if ((Linear_up.getCurrentPosition()>1  || Linear_Up_Dist.getDistance(DistanceUnit.CM)>10.0)) {
                Linear_up.setPower(-1);
                sleep(2300);
                Linear_up.setPower(0);
                sleep(500);
                Linear_Forward.setPower(0);

                sleep(1900);
                Linear_Forward.setPower(0);


                telemetry.addData("ServoBucket:", Servo_bucket.getPower());

                telemetry.addData("Forward encoder value", Linear_Forward.getCurrentPosition());
                telemetry.addData("Up encoder value", Linear_up.getCurrentPosition());
                */


        }
    }
}