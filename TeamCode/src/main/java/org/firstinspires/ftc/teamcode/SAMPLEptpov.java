package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
@Disabled
public class SAMPLEptpov extends LinearOpMode {
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    double left;
    double right;
    double drive;
    double turn;
    double max;
    double slowMode = 0; //0 is off
    double regular_divider=1;
    double slowMode_divider=2;
    float gain=2;
    final float[] hsvValues = new float[3];
    double calibration = 0;//0=off
    DcMotorSimple motorFrontLeft;
    DcMotorSimple motorBackLeft;
    DcMotorSimple motorFrontRight;
    DcMotorSimple motorBackRight;
    DigitalChannel digitalTouch;
    NormalizedColorSensor colorSensor;
    View relativeLayout;
    private DistanceSensor sensorRange;
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
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
        init_controls(true,false,true,true,true,true);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            init_controls(false,false,true,false,true,true);
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            access_pushSensor();
            //calibration
            if (gamepad1.back && calibration==0){
                calibrateColor(true);
            }else if (gamepad1.back && calibration==1){
                calibrateColor(false);
            }
            //
            //slowmode
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
                allPower((int) backLeftPower);
            }else{
                backRightPower /=regular_divider;
                backLeftPower /=regular_divider;
                frontRightPower /=regular_divider;
                frontLeftPower /=regular_divider;
                allPower((int) backLeftPower);
            }
            //
            run_vu();
            sleep(50);
        }
    }
    public void init_controls(boolean update,boolean auto,boolean color_sensor,boolean initial,
                              boolean camera,boolean distance){
        telemetry.addData("Hello", "Driver Lookin good today");
        telemetry.addData("Control", "");
        telemetry.addData("Control", "");
        telemetry.addData("Control", "");
        telemetry.addData("Control", "");
        telemetry.addData("Control", "");
        telemetry.addData("Control", "");
        telemetry.addData("Systems", "Should Be Good To Go");
        if (distance==true){
            telemetry.addData("Distance Sensor", "Running");
            init_distance();
        }
        if (initial){
            init_all();
            if(camera){
                telemetry.addData("Camera", "Running");
                initVuforia();
                initTfod();
            }
        }
        if (color_sensor){
            colorSensorLight();
            init_colorSensor();
            telemetry.addData("Color Sensor", "Running");
            try {
                runSample(); // actually execute the sample
            } finally {
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.WHITE);
                    }
                });
            }
        }
        if (update){
            telemetry.update();
        }else{
            telemetry.addData("Systems", "Running");
        }
        if (!auto){
            telemetry.addData("The Force", "Is With You Driver");
        }else{
            telemetry.addData("Hope", "Auto Works");
        }
    }
    public void init_distance(){
        telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
    }
    public void calibrateColor(boolean on){
        //telemetry.addLine("Higher gain values mean that the sensor
        // will report larger numbers for Red, Green, and Blue, and Value\n");
        if (on) {
            telemetry.addData("In Calibration State", "Press Back to Leave");
            calibration = 1;
            if (gamepad1.dpad_up) {
                gain += 0.005;
            } else if (gamepad1.dpad_down && gain > 1) {
                gain -= 0.005;
            }
        }else if (!on){
            calibration=0;
        }
    }
    public void init_colorSensor(){
        telemetry.addData("Gain", gain);
        colorSensor.setGain(gain);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }
        telemetry.update();
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });
    }
    protected void runSample() {
        float gain = 2;
        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }
    public void colorSensorLight(){
        SwitchableLight light = (SwitchableLight)colorSensor;
        light.enableLight(!light.isLightOn());
    }
    public void access_pushSensor(){
        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }
    }
    public void run_vu(){
        //vuforia
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int i = 0;
                boolean isDuckDetected = false;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    if (recognition.getLabel().equals("Duck")) {
                        isDuckDetected = true;
                        telemetry.addData("Object Detected", "Duck");

                        if (recognition.getLeft() < 200) {
                        } else if (recognition.getLeft() < 400) {
                        } else if (recognition.getLeft() < 600) {
                        } else {
                        }

                    } else {
                    }

                }}}
        //////////////ends vuforia
    }
    public void allPower (int power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }
    public void init_all(){
        robot.init(hardwareMap);
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digital_touch");
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
    }
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    public void initTfod() {
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
