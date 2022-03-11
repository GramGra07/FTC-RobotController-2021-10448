package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="PBAsleep", group="Autonomous")
//@Disabled
public class PBAsleep extends PTPOV {
    HardwarePushbot       robot=new HardwarePushbot();
    private ElapsedTime   runtime = new ElapsedTime();
    double ud;
    double trim;
    public Servo leftClaw;
    /* Declare OpMode members. */
    public DcMotor leftDrive;
    public DcMotor rightDrive;
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
        robot.leftDrive.setPower(-0.20);
        robot.rightDrive.setPower(-0.84);
        sleep(500);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        //forward
        robot.leftDrive.setPower(-0.5);
        robot.rightDrive.setPower(-0.00);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        //forward
        robot.leftDrive.setPower(-0.25);
        robot.rightDrive.setPower(-0.25);
        //sense
        if (digitalTouch.getState()) {
            robot.leftDrive.setPower(-0.25);
            robot.rightDrive.setPower(-0.25);
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            robot.leftDrive.setPower(-0.025);
            robot.rightDrive.setPower(-0.025);
            telemetry.addData("Digital Touch", "Is Pressed");

        }
        //spin
        robot.leftDrive.setPower(-0.025);
        robot.rightDrive.setPower(-0.025);
        sleep(1500);
        robot.rightDrive.setPower(-0.1);
        robot.leftDrive.setPower(-0.1);
        sleep(1500);
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        spinnerL();
        //drift back right
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.0);
        sleep(1000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);


        robot.leftDrive.setPower(-0.35);
        robot.rightDrive.setPower(-.5);
        sleep(1000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setPower(0.25);
        robot.rightDrive.setPower(-.25);
        sleep(100);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        while (getRuntime()<=30) {
            getRuntime();
            telemetry.addData(String.valueOf(runtime), "Working");
        }

    }
    void approachSpinner(){     //facing trucks lined up with trucks in block 2
        driveForward2();
        turnLeft90();
        robot.leftDrive.setPower(.4);
        robot.rightDrive.setPower(.25);
        sleep(850);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    void correctF(){
        robot.leftDrive.setPower(1-trim);
        sleep(100);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    void correctB(){
        robot.leftDrive.setPower(1-trim);
        sleep(100);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    void driveForward2(){
        robot.leftDrive.setPower(-1+trim);
        robot.rightDrive.setPower(-1);
        sleep(300);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    void driveForwardTiny(){
        robot.leftDrive.setPower(-1+trim);
        robot.rightDrive.setPower(-1);
        sleep(20);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        correctF();
    }//works
    void driveBackward1(){
        robot.leftDrive.setPower(1-trim);
        robot.rightDrive.setPower(1);
        sleep(475);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    void turnRight90(){ //520
        robot.leftDrive.setPower(-1);
        robot.rightDrive.setPower(1);
        sleep(275);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    void turnLeft90(){ //520
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(-1);
        sleep(275);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    void spinnerR(){
        robot.leftArm.setPower(-0.3);
        sleep(3800);
        robot.leftArm.setPower(0);
    }//works
    void spinnerL(){
        robot.leftDrive.setPower(-0.04);
        robot.rightDrive.setPower(-0.04);
        robot.leftArm.setPower(0.3);
        sleep(3800);
        robot.leftArm.setPower(0);

    }//works
    void clawOrig(){    //claw to origin from floor
        ud=0.25;
        upDown1.setPower(ud);
        sleep(1300);
        upDown1.setPower(0);
    }
    void clawFloor(){     //from origin
        ud=-0.25;
        upDown1.setPower(ud);
        sleep(1100);
        upDown1.setPower(0);
    }
    void claw1preload(){            //first level from origin
        ud=0.25;
        clawFloor();
        sleep(1100);
        upDown1.setPower(ud);
        sleep(200);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw11(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(300);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw12(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(430);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw1ball(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(360);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw1duck(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(120);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }//end 1
    void claw2preload(){            //first level from origin
        ud=0.25;
        upDown1.setPower(ud);
        sleep(600);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw21(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(1060);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw22(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(1290);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw2ball(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(650);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void claw2duck(){
        ud=0.25;
        upDown1.setPower(ud);
        sleep(540);
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        upDown1.setPower(0);
    }
    void closeClaw(){
        serv = 0;
        leftClaw.setPosition(serv);
        sleep(960);

    }//works
    void openClaw(){
        serv = 1;
        leftClaw.setPosition(serv);
        sleep(960);
    }//works
}

