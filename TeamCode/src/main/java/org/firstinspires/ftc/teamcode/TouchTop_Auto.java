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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Bucket Side Auto Lvl1", group = "Red")
public class TouchTop_Auto extends LinearOpMode {
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor Linear_up = null;
    private DcMotor Linear_Forward = null;
    private CRServo Servo_bucket;
    private Servo servo_arm_1;
    private double Servo_bucket_pos = -0.8;
    private double Servo_bucket_score = 0.6;
    private double Servo_bucket_Touch = 0.9;
    private boolean intake_pos_toggle = true;
    private double intake_pos_low = 0.15;
    private double intake_pos_high = 0.9;
    private int Linear_up_pos = 3000;
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
            Linear_up.setDirection(DcMotor.Direction.REVERSE);
            Linear_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Linear_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Linear_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            while (Linear_Forward.getCurrentPosition() >= -200) {
                Linear_Forward.setTargetPosition(-200);
                Linear_Forward.setPower(-1);
                telemetry.addData("Current Pos-Set", Linear_Forward.getTargetPosition() - Linear_Forward.getCurrentPosition());
                telemetry.update();
            }
            Linear_Forward.setPower(0);
            Linear_up.setPower(0);
            robot.drive(6, 0.80, 0.2);// Push from wall 5 "
            robot.strafe(12.5, 0.80, 0.2); //Strafe infront of Triangle for bucket
            robot.turnTo(-45.0, 0.4, 0.2); // Rotate to line up with bucket
            Linear_up.setTargetPosition(Linear_up_pos);
            while (Linear_up.getCurrentPosition() < Linear_up_pos) {
                Linear_up.setPower(0.8);
            }
            telemetry.addData("Current Pos-Set", Linear_up.getTargetPosition() - Linear_up.getCurrentPosition());
            telemetry.update();

            Linear_up.setPower(0);
            Servo_bucket.setPower(Servo_bucket_score);
            sleep(1500);
            Servo_bucket.setPower(Servo_bucket_pos);
            sleep(500);
            Servo_bucket.setPower(Servo_bucket_Touch);
            robot.drive(20, 0.50, 0.2);
            robot.turnTo(0, 0.4, 0.2);
            robot.drive(36, 0.80, 0.2);
            robot.strafe(-11, 0.80, 0.2);
            robot.turnTo(90, 0.4, 0.2);
            Linear_up.setTargetPosition(10);
            Linear_up.setPower(-.8);
            while (Linear_Up_Dist.getDistance(DistanceUnit.CM) > 10.0) {
                Linear_up.setPower(-1);
            }
            Linear_up.setPower(0);
            servo_arm_1.setPosition(intake_pos_high);


        }
    }
}
