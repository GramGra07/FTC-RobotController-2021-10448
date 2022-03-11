package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="PABsense", group="Autonomous")
//@Disabled
public class PABsense extends PTPOV {
    public double levelRead=0;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    HardwarePushbot       robot=new HardwarePushbot();
    private ElapsedTime   runtime = new ElapsedTime();
    double ud;
    double trim;
    public Servo leftClaw;
    /* Declare OpMode members. */
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor upDown1;
    public  DcMotor upDown2;
    public long upDT=2000;
    DigitalChannel digitalTouch;  // Hardware Device Object
    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
        telemetry.update();
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
        upDown2 = hardwareMap.dcMotor.get("upDown2");
        telemetry.addData("Status", "working");

        telemetry.update();
        getRuntime();
        //actual code
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        boolean isDuckDetected = false;

                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                            //
                            if (recognition.getLabel().equals("Duck")) {
                                isDuckDetected = true;
                                telemetry.addData("Object Detected", "Duck");
                                if (recognition.getLeft() < 200) {
                                    levelRead = 1;
                                } else if (recognition.getLeft() < 400) {
                                    levelRead = 2;
                                } else if (recognition.getLeft() < 600) {
                                    levelRead = 3;
                                } else {
                                    isDuckDetected = false;
                                    telemetry.addData("Not", "Detected");
                                }

                            } else {
                                isDuckDetected = false;
                            }
                        }
                    }
                }
                //differences:go right at beginning, reversed backwards
                if (levelRead==1){
                    telemetry.addData("Level", "1");
                    //lower the arm
                    leftClaw.setPosition(0);
                    telemetry.addData("closing","servo");
                    telemetry.update();
                    sleep(3000);

                    upDown1.setPower(-1);
                    sleep(750);
                    upDown1.setPower(0);
                    //forward with tilt to left
                    leftDrive.setPower(-1);
                    rightDrive.setPower(-0.1);
                    sleep(250);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(-0.25);
                    rightDrive.setPower(-0.25);
                    sleep(1500);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    //open claw
                    sleep(1000);
                    leftClaw.setPosition(1);
                    sleep(2000);
                    leftDrive.setPower(0.25);
                    rightDrive.setPower(0.25);
                    sleep(900);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftClaw.setPosition(0);
                    sleep(1000);
                    //backup and to right
                    leftDrive.setPower(-0.5);
                    rightDrive.setPower(0.5);
                    sleep(650);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    //back
                    overBarrier();
                    //fwOff();
                    break;
                }else if (levelRead==2){
                    telemetry.addData("Level", "2");
                    leftClaw.setPosition(0);
                    telemetry.addData("closing","servo");
                    telemetry.update();
                    sleep(3000);
                    //lower the arm
                    upDown1.setPower(-1);
                    sleep(180);
                    upDown1.setPower(0);
                    //forward with tilt to left
                    //forward with tilt to left
                    leftDrive.setPower(-0.3);
                    rightDrive.setPower(0);
                    sleep(100);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(-1);
                    rightDrive.setPower(0.25);
                    sleep(150);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(-0.25);
                    rightDrive.setPower(-0.25);
                    sleep(2200);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    //open claw
                    sleep(1000);
                    leftClaw.setPosition(1);
                    sleep(2000);
                    leftDrive.setPower(0.25);
                    rightDrive.setPower(0.25);
                    sleep(900);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftClaw.setPosition(0);
                    sleep(1000);
                    //backup and to right
                    leftDrive.setPower(-0.5);
                    rightDrive.setPower(0.5);
                    sleep(600);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    //back
                    overBarrier();
                    //fwOff();
                    break;
                }else if (levelRead==3){
                    telemetry.addData("Level","3");
                    //lower the arm
                    leftClaw.setPosition(0);
                    telemetry.addData("closing","servo");
                    telemetry.update();
                    sleep(3000);
                    //lower the arm
                    upDown1.setPower(-1);
                    sleep(50);
                    upDown1.setPower(0);
                    upDown2.setPower(0.5);
                    sleep(upDT);
                    upDown2.setPower(0);
                    //forward with tilt to left
                    //forward with tilt to left
                    leftDrive.setPower(-1);
                    rightDrive.setPower(-0.1);
                    sleep(250);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftDrive.setPower(-0.25);
                    rightDrive.setPower(-0.25);
                    sleep(1200);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    //open claw
                    sleep(1000);
                    leftClaw.setPosition(1);
                    sleep(2000);
                    leftDrive.setPower(0.25);
                    rightDrive.setPower(0.25);
                    sleep(900);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftClaw.setPosition(0);
                    upDown2.setPower(-0.5);
                    sleep(upDT-1000);
                    upDown2.setPower(0);
                    sleep(1000);
                    //backup and to right
                    leftDrive.setPower(-0.5);
                    rightDrive.setPower(0.5);
                    sleep(600);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    //back
                    overBarrier();
                    //fwOff();
                    break;
                }
                telemetry.update();
            }
            while (getRuntime()<=30) {
                getRuntime();
                telemetry.addData(String.valueOf(runtime), "Working");
            }
        }
    }
    public void fwOff(){
        leftDrive.setPower(-0.15);
        rightDrive.setPower(-0.15);
        sleep(3000);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void overBarrier(){
        leftDrive.setPower(0.6);
        rightDrive.setPower(0.6);
        sleep(1750);
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.3f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}

