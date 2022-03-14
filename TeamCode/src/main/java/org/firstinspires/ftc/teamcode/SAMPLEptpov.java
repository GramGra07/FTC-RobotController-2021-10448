package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.util.List;


@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
@Disabled
public class SAMPLEptpov extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double slowMode = 0; //0 is off
        double regular_divider=1;
        double slowMode_divider=2;
         DcMotor motorFrontLeft;
         DcMotor motorBackLeft;
        DcMotor motorFrontRight;
        DcMotor motorBackRight;



        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
            motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
            motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
            motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
            motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if (gamepad1.a && slowMode==0){
                slowMode=1;
            }else if (gamepad1.a && slowMode==1){
                slowMode=0;
            }
            if (slowMode==1){
                backRightPower /=slowMode_divider;
                backLeftPower /=slowMode_divider;
                frontRightPower /=slowMode_divider;
                frontLeftPower /=slowMode_divider;
            }else{
                backRightPower /=regular_divider;
                backLeftPower /=regular_divider;
                frontRightPower /=regular_divider;
                frontLeftPower /=regular_divider;
            }
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
