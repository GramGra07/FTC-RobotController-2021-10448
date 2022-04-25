package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.List;
import java.util.Locale;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@TeleOp(name="SamplePTPOV", group="Pushbot")
//@Disabled
public class SAMPLEptpov extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    //motors
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    //devices
    DigitalChannel digitalTouch;
    NormalizedColorSensor colorSensor;
    View relativeLayout;
    DistanceSensor distance1;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //slowmode
    double slowMode = 0; //0 is off
    double regular_divider = 1;
    double slowMode_divider = 2;
    //colorSensor
    final float[] hsvValues = new float[3];
    //servo
    public double position = 0;
    public double degree_mult = 0.00277777777;
    //in range
    boolean inRange = false;
    boolean updated_inRange = false;
    boolean updatedHeadingInRange = false;
    //rumble
    boolean endgame = false;                 // Use to prevent multiple half-time warning rumbles.
    Gamepad.RumbleEffect customRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect2;
    Gamepad.RumbleEffect customRumbleEffect3;
    final double End_Game = 75.0;              // Wait this many seconds before rumble-alert for half-time.
    //led
    private final static int LED_PERIOD = 10;//every 10 seconds
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    Telemetry.Item patternName;
    Telemetry.Item display;
    org.firstinspires.ftc.teamcode.SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    //vuforia
    public double levelRead = 0;
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
    //sounds
    String sounds[] = {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie"};
    boolean soundPlaying = false;
    //distance
    public double MM_distance1 = 0;
    public double CM_distance1 = 0;
    public double M_distance1 = 0;
    public double IN_distance1 = 0;
    //color
    public int redVal = 0;
    public int greenVal = 0;
    public int blueVal = 0;
    public String name = "N/A";
    //variable
    public double define = 0; // 0 = off1
    //encoders
    static final double COUNTS_PER_MOTOR_REV = 1200;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = .20;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.6950;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    //telemetry
    public String direction_FW;
    public String direction_LR;
    public String direction_TLR;
    public String slowModeON;
    public String direction_ANGLE;
    public double headingVal=0;
    @Override
    public void runOpMode() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        try {
            runSample(); // actually execute the sample
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
        init_controls(false, false, true, false,
                false, true, true, false, false, true, true);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distance1;
        //sound
        int soundIndex = 0;
        int soundID = -1;
        boolean was_dpad_up = false;
        boolean was_dpad_down = false;
        boolean was_B_down = false;
        Context myApp = hardwareMap.appContext;
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        //
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        composeTelemetry();
        while (opModeIsActive()) {
            //////////flash only works with 2 phones
            showFeedback(false);
            init_controls(false, false, false, false,
                    false, true, true, false, false, false, false);
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            //slowmode
            if (gamepad1.start && gamepad1.back && define == 0 && !was_B_down) {
                define = 1;
            }
            if (gamepad1.start && gamepad1.back && define == 1 && !was_B_down) {
                define = 0;
            }
            if (define == 1) {
                defControllers(true);
            }
            if (gamepad1.b && slowMode == 0) {
                slowMode = 1;
            } else if (gamepad1.b && slowMode == 1) {
                slowMode = 0;
            }
            if (slowMode == 1) {
                backRightPower /= slowMode_divider;
                backLeftPower /= slowMode_divider;
                frontRightPower /= slowMode_divider;
                frontLeftPower /= slowMode_divider;
            } else {
                backRightPower /= regular_divider;
                backLeftPower /= regular_divider;
                frontRightPower /= regular_divider;
                frontLeftPower /= regular_divider;
            }
            //
            ////////sound
            if (gamepad1.dpad_down && !was_dpad_down) {
                soundIndex = (soundIndex + 1) % sounds.length;
            }
            if (gamepad1.dpad_up && !was_dpad_up) {
                soundIndex = (soundIndex + sounds.length - 1) % sounds.length;
            }
            if (gamepad1.a && !soundPlaying) {
                if ((soundID = myApp.getResources().getIdentifier(sounds[soundIndex], "raw", myApp.getPackageName())) != 0) {
                    soundPlaying = true;
                    SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }
                            });
                }
            }
            was_dpad_up = gamepad1.dpad_up;
            was_dpad_down = gamepad1.dpad_down;
            was_B_down = gamepad1.b;
            ////////
            run_vu();
            //endgame init
            if ((runtime.seconds() > End_Game) && !endgame) {
                endGame(true);
            }
            if (!endgame) {
                telemetry.addData(">", "Almost ENDGAME: %3.0f Sec \n", (End_Game - runtime.seconds()));
            }
            //
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            sleep(50);
            teleSpace();
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
            telemetry.addData("Sound >", sounds[soundIndex]);
            telemetry.addData("Status >", soundPlaying ? "Playing" : "Stopped");
            telemetry.update();
        }
    }

    public void dance(int direction) {//-1=back//1=forward
        if (direction == -1) {
            motorFrontLeft.setPower(-direction - 0.2);
            motorBackLeft.setPower(direction);
            motorFrontRight.setPower(-direction - 0.2);
            motorBackRight.setPower(direction);
        }
        if (direction == 1) {
            motorFrontLeft.setPower(direction);
            motorBackLeft.setPower(-direction - 0.2);
            motorFrontRight.setPower(direction);
            motorBackRight.setPower(-direction - 0.2);
        }
    }

    //make space in telemetry read-out
    public void teleSpace() {
        telemetry.addLine()
                .addData("", "");

    }
    public void directionalHeading(){
        headingVal=angles.firstAngle;
        if (headingVal>45 && headingVal<135){
            direction_ANGLE="right";
        }
        if (headingVal<-45 && headingVal<45){
            direction_ANGLE="forward";
        }
        if (headingVal>135 && headingVal>-135){
            direction_ANGLE="backwards";
        }
        if (headingVal>-45 && headingVal<-135){
            direction_ANGLE="left";
        }
    }
//IMPORTANT INIT
    //will initiate all and give names of objects
    public void init_all(boolean motors, boolean servos, boolean color_sensor, boolean distance_sensor) {
        robot.init(hardwareMap);
        if (motors) {
            motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
            motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
            motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
            motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
            motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (servos) {

        }
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digital_touch");

        if (color_sensor) {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        }
        if (distance_sensor) {
            distance1 = hardwareMap.get(DistanceSensor.class, "distance_1");
        }
    }

    //will initiate based on variables and assign variables
    public void init_controls(boolean auto, boolean color_sensor, boolean first,
                              boolean camera, boolean distance, boolean sound, boolean rumble,
                              boolean LED, boolean encoder, boolean imu, boolean controls) {
        telemetry.addData("Hello", "Driver Lookin good today");
        telemetry.addData("Systems", "Should Be Good To Go");
        if (auto) {
            if (encoder) {
                resetEncoder();
                telemetry.addData("Encoders", "Running");
            }
        }
        if (imu) {
            imu();
        }
        if (rumble) {
            init_rumble();
            telemetry.addData("Rumble", "Running");
        }
        if (sound) {
            telemetry.addData("Sound", "Running");
        }
        if (distance) {
            telemetry.addData("Distance Sensor", "Running");
        }
        if (first) {
            init_all(true, false, color_sensor, distance);
            if (camera) {
                telemetry.addData("Camera", "Running");
                initVuforia();
                initTfod();
            }
        }
        if (color_sensor) {
            //colorSensorLight(light);
            init_colorSensor();
            telemetry.addData("Color Sensor", "Running");
        }
        if (!auto) {
            telemetry.addData("The Force", "Is With You Driver");
        } else {
            telemetry.addData("Hope", "Auto Works");
        }
        telemetry.addData("Systems", "Running");
        if (controls) {
            showControls();
        }
        if (LED){
            init_LED();
            telemetry.addData("LED", "Running");
            if (displayKind == org.firstinspires.ftc.teamcode.SampleRevBlinkinLedDriver.DisplayKind.AUTO) {
                doAutoDisplay();
            }
        }
    }

    //controls to be shown on telemetry
    public void showControls(){
        telemetry.addData("Control 1", "Driver");
        telemetry.addData("Control 2", "Other controls");
        telemetry.addData("Control 1", "b = slowmode");
        telemetry.addData("Control 2", "dpad up/down = cycle songs");
        telemetry.addData("Control 2", "A = play song");
        telemetry.addData("Control", "");
    }
    //show controller 1 vs controller 2
    public void defControllers(boolean flash){
        gamepad1.runRumbleEffect(customRumbleEffect3);//1 buzz
        gamepad2.runRumbleEffect(customRumbleEffect2);//2 buzz
        if (flash){
            if (gamepad1.start){
                relativeLayout.setBackgroundColor(0);
                relativeLayout.setBackgroundColor(10);
            }
            if (gamepad2.start){
                relativeLayout.setBackgroundColor(0);
                relativeLayout.setBackgroundColor(10);
                relativeLayout.setBackgroundColor(0);
                relativeLayout.setBackgroundColor(10);
            }
        }
    }
    //telemetry additions
    public void showFeedback(boolean color_sensor){
        if (gamepad1.left_stick_y<0){
            direction_FW="forward";
        }if (gamepad1.left_stick_y>0){
            direction_FW="backward";
        }if (gamepad1.left_stick_y==0){
            direction_FW="idle";
        }
        if (gamepad1.left_stick_x>0){
            direction_LR="right";
        }if (gamepad1.left_stick_x<0){
            direction_LR="left";
        }if (gamepad1.left_stick_x==0){
            direction_LR="idle";
        }
        if (gamepad1.right_stick_x>0){
            direction_TLR="right";
        }if (gamepad1.right_stick_x<0){
            direction_TLR="left";
        }if (gamepad1.right_stick_x==0){
            direction_TLR="idle";
        }
        if (slowMode==1){
            slowModeON= "True";
        }else{
            slowModeON="False";
        }
        directionalHeading();
        telemetry.addLine()
                .addData("direction",   direction_FW)
                .addData("strafe",   direction_LR)
                .addData("turn",direction_TLR)
                .addData("r trigger",  "%.2f", gamepad1.right_trigger)
                .addData("l trigger",  "%.2f", gamepad1.left_trigger);
        teleSpace();
        telemetry.addData("slowMode",slowModeON);
        teleSpace();
        telemetry.addData("Heading","%.1f", angles.firstAngle);
        telemetry.addData("Heading Direction",direction_ANGLE);
        teleSpace();
        if (color_sensor) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue)
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2])
                    .addData("Alpha", "%.3f", colors.alpha);
            get_color_name(colors.red, colors.green, colors.blue);
            telemetry.addLine()
                    .addData("Color", name)
                    .addData("RGB", "(" + redVal + "," + greenVal + "," + blueVal + ")");
            teleSpace();
        }
        access_pushSensor();
        getDistance(true);
        teleSpace();
        //composeTelemetry();//imu
    }
    //gyroscope with heading pitch and roll
    public void imu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
    }
    //endgame effects//dont really work well
    public void endGame(boolean flash){
        gamepad1.runRumbleEffect(customRumbleEffect1);
        gamepad2.runRumbleEffect(customRumbleEffect1);
        endgame =true;
        if (flash){
            relativeLayout.setBackgroundColor(0);
            relativeLayout.setBackgroundColor(10);
            relativeLayout.setBackgroundColor(0);
            relativeLayout.setBackgroundColor(10);
            relativeLayout.setBackgroundColor(0);
            relativeLayout.setBackgroundColor(10);
            relativeLayout.setBackgroundColor(0);
            relativeLayout.setBackgroundColor(10);
            relativeLayout.setBackgroundColor(0);
            relativeLayout.setBackgroundColor(10);
        }
    }
    //range
    //gets the values and finds if it is in a range of max to min
    public void inRange(boolean heading,int maxH,int minH,boolean sensor,int sensor_number,int maxD,int minD,String unit){
        if (heading){
            if (angles.firstAngle>=minH && angles.firstAngle<=maxH){
                updatedHeadingInRange=true;
            }else{
                updatedHeadingInRange=false;
            }
        }
        if (sensor){
            if (sensor_number==1){
                getDistance(false);
                if (unit.equals("cm")){
                    if (CM_distance1>=minD && CM_distance1<=maxD){
                        inRange=true;
                    }
                }
                else if (unit.equals("mm")){
                    if (MM_distance1>=minD && MM_distance1<=maxD){
                        inRange=true;
                    }
                }
                else if (unit.equals("in")){
                    if (IN_distance1>=minD && IN_distance1<=maxD){
                        inRange=true;
                    }
                }
                else if (unit.equals("m")){
                    if (M_distance1>=minD && M_distance1<=maxD){
                        inRange=true;
                    }
                }
                else{
                    inRange=false;
                }
            }
        }
        updateRangeTo(inRange);
    }
    //update range
    public void updateRangeTo(boolean condition){
        updated_inRange= condition;
        inRange=false;
    }
    //resets range
    public void resetRange(){
        updated_inRange= false;
        inRange=false;
    }
    //resets the heading
    public void resetHeading(){
        updatedHeadingInRange= false;
    }
    //imu
    //imu telemetry
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    //colors
    public void init_colorSensor(){
        colorSensor.setGain(10);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });
    }
    public void get_color_name(float red,float green,float blue){
        if ((red<=1) && (red >=0.9375)&& (green<=1)&&(green>=0.8671875) && (blue<=1)&&(blue>=0.67578125)){
            name="white";
        }
        if ((red<=0.5) && (red >=0)&& (green<=1)&&(green>=0.59765625) && (blue<=1)&&(blue>=0.44921875)){
            name="blue";
        }
        if ((red<=0.5) && (red >=0)&& (green<=0.5)&&(green>=0) && (blue<=0.5)&&(blue>=0)){
            name="black";
        }
        if ((red<=1) && (red >=0.3984375)&& (green<=0.234375)&&(green>=0) && (blue<=0.5)&&(blue>=0)){
            name="red";
        }
        getColorRGB(red,green,blue);
    }
    public void getColorRGB(float red,float green, float blue){
        redVal= (int) (red*256);
        greenVal= (int) (green*256);
        blueVal= (int) (blue*256);
    }
    public void runSample() {
        //float gain=2;
        final float[] hsvValues = new float[3];
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }
    //ENCODER
    public void resetEncoder(){
        telemetry.addData("Status", "Resetting Encoders");    //
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.motorFrontRight.getCurrentPosition(),
                robot.motorFrontLeft.getCurrentPosition(),
                robot.motorBackLeft.getCurrentPosition(),
                robot.motorBackRight.getCurrentPosition());
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorFrontLeft .setTargetPosition(-2);
            robot.motorFrontRight.setTargetPosition(-2);
            robot.motorBackLeft  .setTargetPosition(-2);
            robot.motorBackRight .setTargetPosition(-2);
            robot.motorFrontLeft .setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft  .setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight .setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontLeft .setPower(Math.abs(speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorBackLeft  .setPower(Math.abs(speed));
            robot.motorBackRight .setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (robot.motorFrontRight.isBusy() && robot.motorBackRight.isBusy()
                            && robot.motorBackLeft.isBusy() && robot.motorFrontLeft.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorFrontLeft .getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition(),
                        robot.motorBackLeft  .getCurrentPosition(),
                        robot.motorBackRight .getCurrentPosition());
            }
            robot.motorFrontLeft .setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft  .setPower(0);
            robot.motorBackRight .setPower(0);
            robot.motorFrontLeft .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
///////setServo
    public void setServo(int degrees){
        position = degree_mult * degrees;
    }
    //gamepadrumble
    public void init_rumble(){
        customRumbleEffect1 = new Gamepad.RumbleEffect.Builder()//rumble2=right side
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();
        customRumbleEffect2 = new Gamepad.RumbleEffect.Builder()//rumble2=right side
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .build();
        customRumbleEffect3 = new Gamepad.RumbleEffect.Builder()//rumble2=right side
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 300 mSec
                .build();
    }
    //distance
    public void getDistance(boolean give){
        MM_distance1= distance1.getDistance(DistanceUnit.MM);
        CM_distance1= distance1.getDistance(DistanceUnit.CM);
        M_distance1= distance1.getDistance(DistanceUnit.METER);
        IN_distance1= distance1.getDistance(DistanceUnit.INCH);
        //verifyDistance();
        if (give) {
            giveDistances();
        }
    }
    public void verifyDistance(){
        if (MM_distance1*10 !=CM_distance1){
            telemetry.addData("DISTANCE","ERROR");
        }else if (CM_distance1*100 !=M_distance1){
            telemetry.addData("DISTANCE","ERROR");
        //} else if (IN_distance*0.393701 !=CM_distance){
         //   telemetry.addData("DISTANCE","ERROR");
        } else{
            giveDistances();
        }
    }
    public void giveDistances(){
        telemetry.addLine()
            .addData("distance", String.format("%.0001f mm",MM_distance1))
            .addData("distance", String.format("%.0001f cm",CM_distance1))
            .addData("distance", String.format("%.0001f m",M_distance1))
            .addData("distance", String.format("%.0001f in",IN_distance1));
    }
    //push Sensor
    public void access_pushSensor(){
        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }
    }
    //all power
    public void allPower (int power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }
    //vuforia
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
    //Led
    public void init_LED(){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());
        setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.AUTO);
    }
    protected void setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }
    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
}
