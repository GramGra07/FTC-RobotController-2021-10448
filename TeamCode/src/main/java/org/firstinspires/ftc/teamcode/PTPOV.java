package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name="10448 teleOp", group="Pushbot")
//@Disabled
public class PTPOV extends LinearOpMode {
    double left;
    double right;
    double drive;
    double turn;
    double max;
    double turns;
    double ud;
    double trim;
    double serv=0;
    public Servo leftClaw= null;
    public Servo hit= null;
    HardwarePushbot robot           = new HardwarePushbot();   // Use a PushBot's hardware
    double          clawOffset      = 0;                       // Servo mid A
    public DcMotor leftArm;
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor upDown1;
    public  DcMotor upDown2;
    double slowMode=0;
    DigitalChannel digitalTouch;
    DigitalChannel digitalTouch2;
    public double levelRead=0;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");

        robot.init(hardwareMap);
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digital_touch");
        digitalTouch2 = hardwareMap.get(DigitalChannel.class, "digital_touch2");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch2.setMode(DigitalChannel.Mode.INPUT);
        final int CYCLE_MS    =   50;     // period of each cycle/* Initialize the hardware variables.
        // Define and Initialize Motors
        leftClaw = hardwareMap.get(Servo.class, "left_hand");
        hit = hardwareMap.get(Servo.class,"right_hand");
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        upDown1 = hardwareMap.dcMotor.get("upDown1");
        upDown2 = hardwareMap.dcMotor.get("upDown2");
        leftArm = hardwareMap.dcMotor.get("left_arm");

        telemetry.addData("Hello", "Driver Lookin good today");
        telemetry.addData("Control:", "Left stick, right stick = forward/backward,turn"); //
        telemetry.addData("Control:", "RB, LB = open,close");
        telemetry.addData("Control:", "RT, LT = up,down");
        telemetry.addData("Control:", "dpad left, right = spinner");
        telemetry.addData("Control:", "a=slowMode");
        telemetry.addData("Control:", "b=correct");

        telemetry.addData("All Systems","Go" );
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (digitalTouch2.getState()) {

                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
            }
            if (digitalTouch.getState()) {

                telemetry.addData("Digital Touch 2", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch 2", "Is Pressed");
            }

            //slow mode
            if (gamepad1.a && slowMode==0){
                slowMode=1;
            }else if (gamepad1.a && slowMode==1){
                slowMode=0;
            }

            //
            drive = gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;
            //turns thing
            turns = 0;

            ud=0;

            left  = drive + turn;
            right = drive - turn;
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
                turns /= max;
                ud/=max;
            }

            // Output the safe vales to the motor drives.
            if (slowMode==0){
                left=left;
                right=right;
            }else if (slowMode==1){
                left/=2;
                right/=2;
            }
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
            robot.leftArm.setPower(turns);
            if (gamepad1.dpad_right) {
                robot.leftArm.setPower(-0.3);
            }
            if (gamepad1.dpad_left) {
                robot.leftArm.setPower(0.3);
            }


            if (gamepad1.right_trigger>0) {
                //up
                ud = 1;
                upDown1.setPower(ud);

            }else if (gamepad1.left_trigger>0) {
                //down
                ud = -1;
                upDown1.setPower(ud);

            }else{
                ud=0;
                upDown1.setPower(ud);

            }
            if (gamepad1.right_bumper){
                //open
                serv=1;
                leftClaw.setPosition(serv);
            }else if (gamepad1.left_bumper){
                //close
                serv=0;
                leftClaw.setPosition(serv);
            }else if (gamepad1.y){
                leftClaw.setPosition(0.25);
            }else if (gamepad1.x){
                leftClaw.setPosition(0.5);
            }else if (gamepad1.b){
                leftClaw.setPosition(0.75);
            }else{
                leftClaw.setPosition(0.10);
            }

            if (gamepad1.dpad_up){
                upDown2.setPower(0.5);
            } else if (gamepad1.dpad_down){
                upDown2.setPower(-0.5);
            }else{
                upDown2.setPower(0);
            }
            /////////This is vuforia code
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
                            }

                        } else {
                            isDuckDetected = false;
                        }

                    }}}
            //////////////ends vuforia
            sleep(CYCLE_MS);// Send telemetry message to signify robot running;
            telemetry.addData("Control:", "Left stick, right stick = forward/backward,turn"); //
            telemetry.addData("Control:", "RB, LB = open,close");
            telemetry.addData("Control:", "RT, LT = up,down");
            telemetry.addData("Control:", "dpad left, right = spinner");
            telemetry.addData("Control:","a=slowMode");
            telemetry.addData("Control:","b=correct");
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("slowMode","%.2f",slowMode);
            telemetry.addData("Trigger","%2f",ud);
            telemetry.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
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
    }
    void driveForward2(){
        robot.leftDrive.setPower(-1+trim);
        robot.rightDrive.setPower(-1);
        sleep(300);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    void driveBackward1(){
        robot.leftDrive.setPower(1-trim);
        robot.rightDrive.setPower(1);
        sleep(425);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    void turnRight90(){ //520
        robot.leftDrive.setPower(-1);
        robot.rightDrive.setPower(+1);
        sleep(275);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    void turnLeft90(){ //520
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(-1);
        sleep(275 );
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    void spinnerR(){
        robot.leftArm.setPower(0.3);
        sleep(3000);
        robot.leftArm.setPower(0);
    }
    void spinnerL(){
        robot.leftArm.setPower(-0.45);
        sleep(3000);
        robot.leftArm.setPower(0);
    }
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
    }
    void openClaw(){
        serv = 1;
        leftClaw.setPosition(serv);
        sleep(960);
    }
}