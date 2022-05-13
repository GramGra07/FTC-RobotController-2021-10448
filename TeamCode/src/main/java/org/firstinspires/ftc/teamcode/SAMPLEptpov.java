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
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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
//servo

    //vars
    public double position = 0;//sets servo position to 0-1 multiplier
    public double degree_mult = 0.00277777777;//Multiplies this by degrees to see exact position in degrees
//devices
    DigitalChannel digitalTouch;    //push sensor
    NormalizedColorSensor sensor_color;    //color sensor
    View relativeLayout;   //for 2 phones, puts colors on RC
    DistanceSensor distance1;   //distance sensor
    //imu ( inside expansion hub )
    BNO055IMU imu;    //imu module inside expansion hub
    Orientation angles;     //imu uses these to find angles and classify them
    Acceleration gravity;    //Imu uses to get acceleration
    // led
    RevBlinkinLedDriver blinkinLedDriver;//init led driver
    RevBlinkinLedDriver.BlinkinPattern pattern;//led driver pattern
    Telemetry.Item patternName;//shows pattern
    Telemetry.Item display;
    org.firstinspires.ftc.teamcode.SampleRevBlinkinLedDriver.DisplayKind displayKind;//gives display kind between manual and auto
    Deadline ledCycleDeadline;
    //distance
    public double MM_distance1 = 0;//mm distance for distance sensor 1
    public double CM_distance1 = 0;//cm distance for distance sensor 1
    public double M_distance1 = 0;//m distance for distance sensor 1
    public double IN_distance1 = 0;//in distance for distance sensor 1
    //camera
        //vuforia
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";//init tfod
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY = //to actually use vuforia
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //colorSensor
    final float[] hsvValues = new float[3];//gets values for color sensor
    //color
    public int redVal = 0;//the red value in rgb
    public int greenVal = 0;//the green value in rgb
    public int blueVal = 0;//the blue value in rgb
    public String colorName = "N/A";//gets color name
//other vars
    //slowmode
    double slowMode = 0; //0 is off
    double regular_divider = 1;  //tells slowmode how fast to go when not on
    double slowMode_divider = 2; //half speed when slowmode
    //in range
    boolean inRange = false;//tested to see if distance sensor is in range
    boolean updated_inRange = false;//tests again to a boolean for if in range
    boolean updatedHeadingInRange = false;//heading check for low to high
    //rumble
    boolean endgame = false;                 // Use to prevent multiple half-time warning rumbles.
    Gamepad.RumbleEffect customRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect2;//custom rumble
    Gamepad.RumbleEffect customRumbleEffect3;//custom rumble
    final double End_Game = 75.0;              // Wait this many seconds before rumble-alert for half-time.
    //sounds
    String sounds[] = {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie"};
    boolean soundPlaying = false; //finds if the sound is actually playing
    //encoders
    static final double COUNTS_PER_MOTOR_REV = 1200;    // eg: TETRIX Motor Encoder//counts per rotation
    static final double DRIVE_GEAR_REDUCTION = .05;     // This is < 1.0 if geared UP//how much gears
    static final double WHEEL_DIAMETER_INCHES = 4.6950;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);//to get the counts per inch
    //telemetry
    public String direction_FW;//string of direction
    public String direction_LR;//string of direction
    public String direction_TLR;//string of direction
    public String slowModeON;//slowmode string ex(on or off)
    public String direction_ANGLE;//string of angle
    public String pushSensorCheck;//shows value from push sensor
    public double headingVal=0;//heading in degrees
    public double directionPower=0;//power in specified direction
//  ▐▓█▀▀▀▀▀▀▀▀▀█▓▌░▄▄▄▄▄░
//  ▐▓█░░▀░░▀▄░░█▓▌░█▄▄▄█░
//  ▐▓█░░▄░░▄▀░░█▓▌░█▄▄▄█░
//  ▐▓█▄▄▄▄▄▄▄▄▄█▓▌░█████░
//  ░░░░▄▄███▄▄░░░░░█████░
// init vars (used in initiation process)
    public boolean colors=false;//tells to init
    public boolean camera=false;//tells to init
    public boolean distance=false;//tells to init
    public boolean sound=false;//tells to init
    public boolean imuInit=true;//tells to init
    public boolean LED=false;//tells to init
    public boolean push=false;//tells to init
    public boolean picture=false;
    @Override
    public void runOpMode() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        //relative layout is the background on the RC phone which can be used to test color sensors
        if (colors){//if colors is being used
            try {
                runSample(); // actually execute the sample
            } finally {
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.WHITE);//RC phone background
                 }
                });
            }
        }
        init_controls(true,true,false,true);//initiates everything
        if (camera){
            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1, 16.0 / 9.0);
            }
        }
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distance1;//helps init distance sensors
        //sound
        int soundIndex = 0;//gets value in list of sounds
        int soundID;
        Context myApp = hardwareMap.appContext;//gets app content
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();//play new song
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        boolean was_dpad_up = false;//checks if dpad was pressed
        boolean was_dpad_down = false;//checks if dpad was pressed
        boolean was_B_down = false;//checks if b was pressed
        //
        ElapsedTime runtime = new ElapsedTime();//runtime helps with endgame initiation and cues
        waitForStart();
        if (imuInit){//will set up everything for imu
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            composeTelemetry();
        }
        while (opModeIsActive()) {
            init_controls(false,true,false,true);//only imu if first init//initiates everything
            double y = gamepad1.left_stick_y; // Remember, this is reversed!//forward backward
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing//left right
            double rx = -gamepad1.right_stick_x;//turning
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);//gets max value
            double frontLeftPower = (y + x + rx)/denominator;
            double backLeftPower  = (y - x + rx)/denominator;
            double frontRightPower= (y - x - rx)/denominator;
            double backRightPower = (y + x - rx)/denominator;
            //slowmode
            if (gamepad1.b && slowMode == 0) {
                slowMode = 1;//sets slowmode to on
            } else if (gamepad1.b && slowMode == 1) {
                slowMode = 0;//sets slowmode to off
            }
            if (slowMode == 1) {
                backRightPower /= slowMode_divider;//divides power by the divider
                backLeftPower /= slowMode_divider;//divides power by the divider
                frontRightPower /= slowMode_divider;//divides power by the divider
                frontLeftPower /= slowMode_divider;//divides power by the divider
            } else {
                backRightPower /= regular_divider;
                backLeftPower /= regular_divider;
                frontRightPower /= regular_divider;
                frontLeftPower /= regular_divider;
            }
            //
            ////////sound
            if (sound){
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
            }
            was_dpad_up = gamepad1.dpad_up;
            was_dpad_down = gamepad1.dpad_down;
            was_B_down = gamepad1.b;
            //camera run
            if (camera){//will run vuforia
                run_vu();
            }
            //endgame init
            if ((runtime.seconds() > End_Game) && !endgame) {//sets endgame
                endGame(true);
            }
            if (!endgame) {
                telemetry.addData(">", "Almost ENDGAME: %3.0f Sec \n", (End_Game - runtime.seconds()));//shows time til endgame
            }
            //
            //sets power to respective motors
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            sleep(50);
            teleSpace();//puts a space in telemetry
            if ( distance){
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));//distance sensor
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));//distance sensor
            }
            if(sound){
                telemetry.addData("Sound >", sounds[soundIndex]);
                telemetry.addData("Status >", soundPlaying ? "Playing" : "Stopped");
            }
            if (picture) {
                picInTele(0);
            }
            telemetry.update();
        }
    }
    //setServo//this sets the servo to a position based off a given degree
    public double setServo(int degrees){
        position = degree_mult * degrees;
        return position;
    }
    public void dance(String direction_1) {//-1=back//1=forward//fun little thing we learned from others
        directionPower=1;
        if (direction_1.equals("backwards")) {
            motorFrontLeft.setPower(directionPower);
            motorBackLeft.setPower(-directionPower);
            motorFrontRight.setPower(directionPower);
            motorBackRight.setPower(-directionPower);
        }
        if (direction_1.equals("forwards")) {
            motorFrontLeft.setPower(-directionPower);
            motorBackLeft.setPower(directionPower);
            motorFrontRight.setPower(-directionPower);
            motorBackRight.setPower(directionPower);
        }
    }
    //make space in telemetry read-out
    public void teleSpace() {
        telemetry.addLine();
    }
//IMPORTANT INIT
    //will initiate all and give names of objects
    public void init_all() {
        robot.init(hardwareMap);
        //init all motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        if (push){//init push sensor
            digitalTouch = hardwareMap.get(DigitalChannel.class, "digital_touch");
        }
        if (colors) {//init all color things
            sensor_color = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        }
        if (distance) {//initiates all distance data
            distance1 = hardwareMap.get(DistanceSensor.class, "distance_1");
        }
    }
    //will initiate based on variables and assign variables
    public void init_controls(boolean first, boolean rumble, boolean encoder, boolean controls) {
        showFeedback();//gives feedback on telemetry
        telemetry.addData("Hello", "Driver Lookin good today");
        telemetry.addData("Systems", "Should Be Good To Go");
        if (rumble) {//gamepad rumble
            init_rumble();
            telemetry.addData("Rumble", "Running");
        }
        if (sound) {//sounds
            telemetry.addData("Sound", "Running");
        }
        if (distance) {
            telemetry.addData("Distance Sensor", "Running");
        }
        //only on first initiation
        if (first) {
            init_all();
            if (camera) {//only on first init and if the camera variable is true
                telemetry.addData("Camera", "Running");
                initVuforia();
                initTfod();
            }
            if (encoder) {//only on first init and if the encoder variable is true
                resetEncoder();
                telemetry.addData("Encoders", "Running");
            }
            if (imuInit){
                imu();
            }
        }
        if (colors) {//will init the color sensor values
            init_colorSensor();
            telemetry.addData("Color Sensor", "Running");
        }
        if (LED){
            init_LED();
            telemetry.addData("LED", "Running");
            if (displayKind == org.firstinspires.ftc.teamcode.SampleRevBlinkinLedDriver.DisplayKind.AUTO) {
                doAutoDisplay();
            }
        }
        telemetry.addData("Systems", "Running");
        if (controls) {
            showControls();
        }
    }

    //controls to be shown on telemetry
    public void showControls(){
        telemetry.addData("Control 1", "Driver");
        telemetry.addData("Control 2", "Other controls");
        telemetry.addData("Control 1", "b = slowmode");
        telemetry.addData("Control 2", "dpad up/down = cycle songs");
        telemetry.addData("Control 2", "A = play song");
    }
    //telemetry additions
    public void showFeedback(){
    //get variables for telemetry
        //gets direction vertical
        if (gamepad1.left_stick_y<0){
            direction_FW="forward";
        }if (gamepad1.left_stick_y>0){
            direction_FW="backward";
        }if (gamepad1.left_stick_y==0){
            direction_FW="idle";
        }
        //gets direction horizontal
        if (gamepad1.left_stick_x>0){
            direction_LR="right";
        }if (gamepad1.left_stick_x<0){
            direction_LR="left";
        }if (gamepad1.left_stick_x==0){
            direction_LR="idle";
        }
        //gets turn angle
        if (gamepad1.right_stick_x>0){
            direction_TLR="right";
        }if (gamepad1.right_stick_x<0){
            direction_TLR="left";
        }if (gamepad1.right_stick_x==0){
            direction_TLR="idle";
        }
        //shows slowmode status
        if (slowMode==1){
            slowModeON= "True";
        }else{
            slowModeON="False";
        }
        //direction heading
        if (imuInit){
            headingVal=angles.firstAngle;//sets angle to var to check
            telemetry.addData("Heading","%.1f", angles.firstAngle);
            telemetry.addData("Heading Direction",direction_ANGLE);
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
        //checks push sensor
        if(push){
            if (digitalTouch.getState()) {
                pushSensorCheck="Not Pressed";
            } else {
                pushSensorCheck="Pressed";
            }
        }
    //shows all previously defined values
        telemetry.addLine()
                .addData("direction",   direction_FW)
                .addData("strafe",   direction_LR)
                .addData("turn",direction_TLR)
                .addData("r trigger",  "%.2f", gamepad1.right_trigger)
                .addData("l trigger",  "%.2f", gamepad1.left_trigger);
        teleSpace();
        telemetry.addData("slowMode",slowModeON);
        teleSpace();
        //gives color values
        if (colors){
            NormalizedRGBA colors = sensor_color.getNormalizedColors();
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
                    .addData("Color", colorName)
                    .addData("RGB", "(" + redVal + "," + greenVal + "," + blueVal + ")");//shows rgb value
        }
        teleSpace();
        if (push) {
            telemetry.addData("Digital Touch", pushSensorCheck);
        }
        if(distance){
            getDistance1(true);//gets and shows distances
        }
        teleSpace();
    }
//ENDS OVERALL INIT PROCESS
//INIT ALL DIFFERENT VALUES AND DEVICES
//imu
    public void imu(){//gyroscope with heading pitch and roll
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
//colors
    public void init_colorSensor(){
        sensor_color.setGain(10);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });
    }
    public void get_color_name(float red,float green,float blue){
        if ((red<=1) && (red >=0.9375)&& (green<=1)&&(green>=0.8671875) && (blue<=1)&&(blue>=0.67578125)){
            colorName="white";
        }
        if ((red<=0.5) && (red >=0)&& (green<=1)&&(green>=0.59765625) && (blue<=1)&&(blue>=0.44921875)){
            colorName="blue";
        }
        if ((red<=0.5) && (red >=0)&& (green<=0.5)&&(green>=0) && (blue<=0.5)&&(blue>=0)){
            colorName="black";
        }
        if ((red<=1) && (red >=0.3984375)&& (green<=0.234375)&&(green>=0) && (blue<=0.5)&&(blue>=0)){
            colorName="red";
        }
        getColorRGB(red,green,blue);
    }
    public void getColorRGB(float red,float green, float blue){
        redVal= (int) (red*256);
        greenVal= (int) (green*256);
        blueVal= (int) (blue*256);
    }
    public void runSample() {
        if (sensor_color instanceof SwitchableLight) {
            ((SwitchableLight) sensor_color).enableLight(true);
        }
    }
//encoder
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
//distance
    public void getDistance1(boolean give){
        MM_distance1= distance1.getDistance(DistanceUnit.MM);
        CM_distance1= distance1.getDistance(DistanceUnit.CM);
        M_distance1= distance1.getDistance(DistanceUnit.METER);
        IN_distance1= distance1.getDistance(DistanceUnit.INCH);
        //verifyDistance();
        if (give) {
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
//camera
    public void run_vu(){
        if (tfod != null) {//not nothing
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

                }
            }
        }
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
//END OVERALL INIT FUNCTIONS
    //endgame effects
    public void endGame(boolean flash){
        gamepad1.runRumbleEffect(customRumbleEffect1);//gives a custom rumble effect to both gamepads
        gamepad2.runRumbleEffect(customRumbleEffect1);
        endgame =true;
        if (flash){
            flash();
        }
    }
    public void flash(){//will flash the RC screen
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
    //range
    //gets the values and finds if it is in a range of max to min
    public void inRange(boolean heading,int maxH,int minH,boolean sensor,int sensor_number,int maxD,int minD,String unit){
        resetRanges();
        if (heading){
            if (angles.firstAngle>=minH && angles.firstAngle<=maxH){
                updatedHeadingInRange=true;
            }else{
                updatedHeadingInRange=false;
            }
        }
        if (sensor){
            if (sensor_number==1){
                getDistance1(false);
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
    public void resetRanges(){
        updated_inRange= false;
        inRange=false;
    }
    //resets the heading
    public void resetHeading(){
        updatedHeadingInRange= false;
    }
//encoder driving
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
    //put cool thing in telemetry
    public void picInTele(int choice){
        if (choice==0){
            teleSpace();
        }
        if (choice==1){
            telemetry.addLine().addData("▐▓█▀▀▀▀▀▀▀▀▀█▓▌░▄▄▄▄▄░                      ","");
            telemetry.addLine().addData("▐▓█░░▀░░▀▄░░█▓▌░█▄▄▄█░                      ","");
            telemetry.addLine().addData("▐▓█░░▄░░▄▀░░█▓▌░█▄▄▄█░                      ","");
            telemetry.addLine().addData("▐▓█▄▄▄▄▄▄▄▄▄█▓▌░█████░                      ","");
            telemetry.addLine().addData("░░░░▄▄███▄▄░░░░░█████░                      ","");
        }
        if (choice==2){
            telemetry.addLine().addData("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$                      ","");
            telemetry.addLine().addData("$$$$$$$$$$$$$$$$$$$$$$$$_____$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$$$$$$$$$$$$$$$_____$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$$$$$$$$$$$$$$$_____$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$$____$$$____$$_____$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$______$______$_____$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$______$______$_____$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$____$$$$$$$$$$$$$$$$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$___$$___________$$$$$$$                      ","");
            telemetry.addLine().addData("$$$$_____$__$$_______________$$$$                      ","");
            telemetry.addLine().addData("$$$$__________$$_____________$$$$                      ","");
            telemetry.addLine().addData("$$$$___________$$___________$$$$$                      ","");
            telemetry.addLine().addData("$$$$_____________$_________$$$$$$                      ","");
            telemetry.addLine().addData("$$$$$_____________________$$$$$$$                      ","");
            telemetry.addLine().addData("$$$$$$___________________$$$$$$$$                      ","");
            telemetry.addLine().addData("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$                      ","");
        }
        if (choice==3){
            telemetry.addLine().addData("─▄▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▄                       ","");
            telemetry.addLine().addData("█░░░█░░░░░░░░░░▄▄░██░█                      ","");
            telemetry.addLine().addData("█░▀▀█▀▀░▄▀░▄▀░░▀▀░▄▄░█                      ","");
            telemetry.addLine().addData("█░░░▀░░░▄▄▄▄▄░░██░▀▀░█                      ","");
            telemetry.addLine().addData("─▀▄▄▄▄▄▀─────▀▄▄▄▄▄▄▀                       ","");
        }
        if (choice==4){
            telemetry.addLine().addData("──▄────▄▄▄▄▄▄▄────▄───                      ","");
            telemetry.addLine().addData("─▀▀▄─▄█████████▄─▄▀▀──                      ","");
            telemetry.addLine().addData("─────██─▀███▀─██──────                      ","");
            telemetry.addLine().addData("───▄─▀████▀████▀─▄────                      ","");
            telemetry.addLine().addData("─▀█────██▀█▀██────█▀──                      ","");
        }
        if (choice==5){
            telemetry.addLine().addData("───▄█▌─▄─▄─▐█▄───                         ","");
            telemetry.addLine().addData("───██▌▀▀▄▀▀▐██───                         ","");
            telemetry.addLine().addData("───██▌─▄▄▄─▐██───                         ","");
            telemetry.addLine().addData("───▀██▌▐█▌▐██▀───                         ","");
            telemetry.addLine().addData("▄██████─▀─██████▄                         ","");
        }
        if (choice==6){
            telemetry.addLine().addData("░░░░░░░▄█▄▄▄█▄░░░░░░░                      ","");
            telemetry.addLine().addData("▄▀░░░░▄▌─▄─▄─▐▄░░░░▀▄                      ","");
            telemetry.addLine().addData("█▄▄█░░▀▌─▀─▀─▐▀░░█▄▄█                      ","");
            telemetry.addLine().addData("░▐▌░░░░▀▀███▀▀░░░░▐▌░                      ","");
            telemetry.addLine().addData("████░▄█████████▄░████                      ","");
        }
        if (choice==7){
            telemetry.addLine().addData("╭━┳━╭━╭━╮╮                             ","");
            telemetry.addLine().addData("┃┈┈┈┣▅╋▅┫┃                            ","");
            telemetry.addLine().addData("┃┈┃┈╰━╰━━━━━━╮                        ","");
            telemetry.addLine().addData("╰┳╯┈┈┈┈┈┈┈┈┈◢▉◣                      ","");
            telemetry.addLine().addData("╲┃┈┈┈┈┈┈┈┈┈┈▉▉▉                       ","");
            telemetry.addLine().addData("╲┃┈┈┈┈┈┈┈┈┈┈◥▉◤                      ","");
            telemetry.addLine().addData("╲┃┈┈┈┈╭━┳━━━━╯                        ","");
            telemetry.addLine().addData("╲┣━━━━━━┫                             ","");
        }
        if (choice==8){
            telemetry.addLine().addData("______________$$$$$$$                                           ","");
            telemetry.addLine().addData("_____________$$$$$$$$$                                          ","");
            telemetry.addLine().addData("____________$$$$$$$$$$$                                         ","");
            telemetry.addLine().addData("____________$$$$$$$$$$$                                         ","");
            telemetry.addLine().addData("____________$$$$$$$$$$$                                         ","");
            telemetry.addLine().addData("_____________$$$$$$$$$                                          ","");
            telemetry.addLine().addData("_____$$$$$$_____$$$$$$$$$$                                      ","");
            telemetry.addLine().addData("____$$$$$$$$__$$$$$$_____$$$                                    ","");
            telemetry.addLine().addData("___$$$$$$$$$$$$$$$$_________$                                   ","");
            telemetry.addLine().addData("___$$$$$$$$$$$$$$$$______$__$                                   ","");
            telemetry.addLine().addData("___$$$$$$$$$$$$$$$$_____$$$_$                                   ","");
            telemetry.addLine().addData("___$$$$$$$$$$$__________$$$_$_____$$                            ","");
            telemetry.addLine().addData("____$$$$$$$$$____________$$_$$$$_$$$$                           ","");
            telemetry.addLine().addData("______$$$__$$__$$$______________$$$$                            ","");
            telemetry.addLine().addData("___________$$____$_______________$                              ","");
            telemetry.addLine().addData("____________$$____$______________$                              ","");
            telemetry.addLine().addData("_____________$$___$$$__________$$                               ","");
            telemetry.addLine().addData("_______________$$$_$$$$$$_$$$$$                                 ","");
            telemetry.addLine().addData("________________$$____$$_$$$$$                                  ","");
            telemetry.addLine().addData("_______________$$$$$___$$$$$$$$$$                               ","");
            telemetry.addLine().addData("_______________$$$$$$$$$$$$$$$$$$$$                             ","");
            telemetry.addLine().addData("_______________$$_$$$$$$$$$$$$$$__$$                            ","");
            telemetry.addLine().addData("_______________$$__$$$$$$$$$$$___$_$                            ","");
            telemetry.addLine().addData("______________$$$__$___$$$______$$$$                            ","");
            telemetry.addLine().addData("______________$$$_$__________$$_$$$$                            ","");
            telemetry.addLine().addData("______________$$$$$_________$$$$_$_$                            ","");
            telemetry.addLine().addData("_______________$$$$__________$$$__$$                            ","");
            telemetry.addLine().addData("_____$$$$_________$________________$                            ","");
            telemetry.addLine().addData("___$$$___$$______$$$_____________$$                             ","");
            telemetry.addLine().addData("__$___$$__$$_____$__$$$_____$$__$$                              ","");
            telemetry.addLine().addData("_$$____$___$_______$$$$$$$$$$$$$                                ","");
            telemetry.addLine().addData("_$$_____$___$_____$$$$$_$$___$$$                                ","");
            telemetry.addLine().addData("_$$_____$___$___$$$$____$____$$                                 ","");
            telemetry.addLine().addData("__$_____$$__$$$$$$$____$$_$$$$$                                 ","");
            telemetry.addLine().addData("__$$_____$___$_$$_____$__$__$$$$$$$$$$$$                        ","");
            telemetry.addLine().addData("___$_____$$__$_$_____$_$$$__$$__$______$$$                      ","");
            telemetry.addLine().addData("____$$_________$___$$_$___$$__$$_________$                      ","");
            telemetry.addLine().addData("_____$$_$$$$___$__$$__$__________________$                      ","");
            telemetry.addLine().addData("______$$____$__$$$____$__________________$                      ","");
            telemetry.addLine().addData("_______$____$__$_______$$______________$$                       ","");
            telemetry.addLine().addData("_______$$$$_$$$_________$$$$$$$__$$$$$$                         ","");
            telemetry.addLine().addData("__________$$$_________________$$$$$                             ","");
        }
    }
    //5lines
//  ▐▓█▀▀▀▀▀▀▀▀▀█▓▌░▄▄▄▄▄░
//  ▐▓█░░▀░░▀▄░░█▓▌░█▄▄▄█░
//  ▐▓█░░▄░░▄▀░░█▓▌░█▄▄▄█░
//  ▐▓█▄▄▄▄▄▄▄▄▄█▓▌░█████░
//  ░░░░▄▄███▄▄░░░░░█████░
    //16lines
//  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//  $$$$$$$$$$$$$$$$$$$$$$$$_____$$$$
//  $$$$_____$$$$$$$$$$$$$$$_____$$$$
//  $$$$_____$$$$$$$$$$$$$$$_____$$$$
//  $$$$_____$$____$$$____$$_____$$$$
//  $$$$_____$______$______$_____$$$$
//  $$$$_____$______$______$_____$$$$
//  $$$$_____$____$$$$$$$$$$$$$$$$$$$
//  $$$$_____$___$$___________$$$$$$$
//  $$$$_____$__$$_______________$$$$
//  $$$$__________$$_____________$$$$
//  $$$$___________$$___________$$$$$
//  $$$$_____________$_________$$$$$$
//  $$$$$_____________________$$$$$$$
//  $$$$$$___________________$$$$$$$$
//  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    //5lines
//  ─▄▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▄
//  █░░░█░░░░░░░░░░▄▄░██░█
//  █░▀▀█▀▀░▄▀░▄▀░░▀▀░▄▄░█
//  █░░░▀░░░▄▄▄▄▄░░██░▀▀░█
//  ─▀▄▄▄▄▄▀─────▀▄▄▄▄▄▄▀
    //5lines
//  ──▄────▄▄▄▄▄▄▄────▄───
//  ─▀▀▄─▄█████████▄─▄▀▀──
//  ─────██─▀███▀─██──────
//  ───▄─▀████▀████▀─▄────
//  ─▀█────██▀█▀██────█▀──
    //5lines
//  ───▄█▌─▄─▄─▐█▄
//  ───██▌▀▀▄▀▀▐██
//  ───██▌─▄▄▄─▐██
//  ───▀██▌▐█▌▐██▀
//  ▄██████─▀─██████▄
    //5lines
//  ░░░░░░░▄█▄▄▄█▄░░░░░░░
//  ▄▀░░░░▄▌─▄─▄─▐▄░░░░▀▄
//  █▄▄█░░▀▌─▀─▀─▐▀░░█▄▄█
//  ░▐▌░░░░▀▀███▀▀░░░░▐▌░
//  ████░▄█████████▄░████
    //8lines
//  ╭━┳━╭━╭━╮╮
//  ┃┈┈┈┣▅╋▅┫┃
//  ┃┈┃┈╰━╰━━━━━━╮
//  ╰┳╯┈┈┈┈┈┈┈┈┈◢▉◣
//  ╲┃┈┈┈┈┈┈┈┈┈▉▉▉
//  ╲┃┈┈┈┈┈┈┈┈┈◥▉◤
//  ╲┃┈┈┈┈╭━┳━━━━╯
//  ╲┣━━━━━━┫
    //42lines
//  ______________$$$$$$$
//  _____________$$$$$$$$$
//  ____________$$$$$$$$$$$
//  ____________$$$$$$$$$$$
//  ____________$$$$$$$$$$$
//  _____________$$$$$$$$$
//  _____$$$$$$_____$$$$$$$$$$
//  ____$$$$$$$$__$$$$$$_____$$$
//  ___$$$$$$$$$$$$$$$$_________$
//  ___$$$$$$$$$$$$$$$$______$__$
//  ___$$$$$$$$$$$$$$$$_____$$$_$
//  ___$$$$$$$$$$$__________$$$_$_____$$
//  ____$$$$$$$$$____________$$_$$$$_$$$$
//  ______$$$__$$__$$$______________$$$$
//  ___________$$____$_______________$
//  ____________$$____$______________$
//  _____________$$___$$$__________$$
//  _______________$$$_$$$$$$_$$$$$
//  ________________$$____$$_$$$$$
//  _______________$$$$$___$$$$$$$$$$
//  _______________$$$$$$$$$$$$$$$$$$$$
//  _______________$$_$$$$$$$$$$$$$$__$$
//  _______________$$__$$$$$$$$$$$___$_$
//  ______________$$$__$___$$$______$$$$
//  ______________$$$_$__________$$_$$$$
//  ______________$$$$$_________$$$$_$_$
//  _______________$$$$__________$$$__$$
//  _____$$$$_________$________________$
//  ___$$$___$$______$$$_____________$$
//  __$___$$__$$_____$__$$$_____$$__$$
//  _$$____$___$_______$$$$$$$$$$$$$
//  _$$_____$___$_____$$$$$_$$___$$$
//  _$$_____$___$___$$$$____$____$$
//  __$_____$$__$$$$$$$____$$_$$$$$
//  __$$_____$___$_$$_____$__$__$$$$$$$$$$$$
//  ___$_____$$__$_$_____$_$$$__$$__$______$$$
//  ____$$_________$___$$_$___$$__$$_________$
//  _____$$_$$$$___$__$$__$__________________$
//  ______$$____$__$$$____$__________________$
//  _______$____$__$_______$$______________$$
//  _______$$$$_$$$_________$$$$$$$__$$$$$$
//  __________$$$_________________$$$$$
}
