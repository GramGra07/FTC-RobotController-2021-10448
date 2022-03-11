package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="STAY", group="Autonomous")
//@Disabled
public class STAY extends PTPOV {
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
        while (getRuntime()<=30) {
            getRuntime();
            telemetry.addData(String.valueOf(runtime), "Working");
        }

    }
    public void approachSpinner(){     //facing trucks lined up with trucks in block 2
        driveForward2();
        turnLeft90();
        robot.leftDrive.setPower(.4);
        robot.rightDrive.setPower(.25);
        sleep(850);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    public void correctF(){
        robot.leftDrive.setPower(1-trim);
        sleep(100);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    public void correctB(){
        robot.leftDrive.setPower(1-trim);
        sleep(100);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    public void driveForward2(){
        robot.leftDrive.setPower(-1+trim);
        robot.rightDrive.setPower(-1);
        sleep(300);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    public void driveForwardTiny(){
        robot.leftDrive.setPower(-1+trim);
        robot.rightDrive.setPower(-1);
        sleep(20);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        correctF();
    }//works
    public void driveBackward1(){
        robot.leftDrive.setPower(1-trim);
        robot.rightDrive.setPower(1);
        sleep(475);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    public void turnRight90(){ //520
        robot.leftDrive.setPower(-1);
        robot.rightDrive.setPower(1);
        sleep(275);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    public void turnLeft90(){ //520
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(-1);
        sleep(275);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }//works
    public void spinnerR(){
        robot.leftArm.setPower(-0.3);
        sleep(3000);
        robot.leftArm.setPower(0);
    }//works
    public void spinnerL(){
        robot.leftArm.setPower(0.3);
        sleep(3000);
        robot.leftArm.setPower(0);
    }//works
    public void clawOrig(){    //claw to origin from floor
        ud=0.25;
        upDown1.setPower(ud);

        sleep(1300);
        upDown1.setPower(0);
    }
    public void clawFloor(){     //from origin
        ud=-0.25;
        upDown1.setPower(ud);

        sleep(1100);
        upDown1.setPower(0);
    }
    public void claw1preload(){            //first level from origin
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
    public void claw11(){
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
    public void claw12(){
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
    public void claw1ball(){
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
    public void claw1duck(){
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
    public void claw2preload(){            //first level from origin
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
    public void claw21(){
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
    public void claw22(){
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
    public void claw2ball(){
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
    public void claw2duck(){
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
    public void closeClaw(){
        serv = 0;
        leftClaw.setPosition(serv);
        sleep(960);

    }//works
    public void openClaw(){
        serv = 1;
        leftClaw.setPosition(serv);
        sleep(960);
    }//works
}

