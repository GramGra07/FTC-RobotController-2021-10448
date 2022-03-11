package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="PABWsleep", group="Autonomous")
//@Disabled
public class PABWsleep extends PTPOV {
    HardwarePushbot       robot=new HardwarePushbot();
    private ElapsedTime   runtime = new ElapsedTime();
    double ud;
    double trim;
    public Servo leftClaw;
    /* Declare OpMode members. */
    public DcMotor leftDrive;
    public DcMotor rightDrive;  // makes pp shoot lasers
    public DcMotor upDown1;
    DigitalChannel digitalTouch;  // Hardware Device Object
    @Override
    public void runOpMode(){
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digital_touch");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        telemetry.update();
        runtime.reset();
        getRuntime();
        robot.init(hardwareMap);
        waitForStart();
        leftClaw = hardwareMap.get(Servo.class, "left_hand");
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        upDown1 = hardwareMap.dcMotor.get("upDown1");
        telemetry.addData("Status", "working");

        telemetry.update();
        getRuntime();
        //actual code
        //forward
        leftDrive.setPower(-1);
        rightDrive.setPower(-1);
        sleep(30);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //turn
        leftDrive.setPower(0);
        rightDrive.setPower(-1);
        sleep(500);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //over barrier
        leftDrive.setPower(-0.4);
        rightDrive.setPower(-0.4);
        sleep(3000);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        while (getRuntime()<=30) {
            getRuntime();
            telemetry.addData(String.valueOf(runtime), "Working");
        }

    }

}

