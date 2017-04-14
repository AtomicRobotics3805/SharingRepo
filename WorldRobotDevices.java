package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AdafruitIMU;
import org.firstinspires.ftc.teamcode.LightTest;

/**
 * World robot device initialization class
 */

public class WorldRobotDevices {
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor collectionMotor = null;
    public DcMotor flickerMotor = null;
    public DcMotor liftLeft = null;
    public DcMotor liftRight = null;

    public Servo ballIndexer = null;
    public Servo leftButtonPushers = null;
    public Servo rightButtonPushers = null;
    public Servo frontPusher = null;
    public Servo liftRelease = null;
    public Servo capHold = null;

    //Sensors
    public AdafruitIMU imu;
    public LightSensor leftLightSensor;
    public LightSensor rightLightSensor;
    public AnalogInput leftSharpIR;
    public AnalogInput rightSharpIR;
    /*public AnalogInput leftUltrasonic;
    public AnalogInput rightUltrasonic;*/
    public ColorSensor rightFrontColor;
    public ColorSensor rightBackColor;
    public ColorSensor leftFrontColor;
    public ColorSensor leftBackColor;

    //Sensor comparison values
    public final double RIGHT_LIGHT_SENSOR_THRESHOLD = 0.304;
    public final double LEFT_LIGHT_SENSOR_THRESHOLD = 0.285;

    //Servo position constants
    public final double BIS_IDLE = 0.5, BIS_LOAD = 0, BIS_REVERSE = 1;
    public final double LBPS_CENTER = 0.580, LBPS_LEFT = 0.995, LBPS_RIGHT = 0.165;
    public final double RBPS_CENTER = 0.485, RBPS_LEFT = 0.070, RBPS_RIGHT = 0.900;
    public final double LRS_HOLD = 0.7508, LRS_RELEASE = 0.3150;
    public final double FPS_HOLD = 0.7214, FPS_RELEASE = 0.5082;
    public final double CBHS_IDLE = 0.8311, CBHS_GRIP = 0;

    //Private variables
    private double wallKp = 0.05;
    private double wallKi = 0.00001;
    private double wallKd = 0.1;
    private double wallIntegral;
    private double wallPreError;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    OpMode opmode;

    /* Constructor */
    public WorldRobotDevices() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, OpMode aopmode) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        opmode = aopmode;

        // Define and Initialize Motors and Servos
        leftFrontMotor = hwMap.dcMotor.get("LFM");
        rightFrontMotor = hwMap.dcMotor.get("RFM");
        leftBackMotor = hwMap.dcMotor.get("LBM");
        rightBackMotor = hwMap.dcMotor.get("RBM");
        collectionMotor = hwMap.dcMotor.get("CM");
        flickerMotor = hwMap.dcMotor.get("FM");
        liftLeft = hwMap.dcMotor.get("LLM");
        liftRight = hwMap.dcMotor.get("RRM");

        ballIndexer = hwMap.servo.get("BIS");
        leftButtonPushers = hwMap.servo.get("LBPS");
        rightButtonPushers = hwMap.servo.get("RBPS");
        frontPusher = hwMap.servo.get("FPS");
        liftRelease = hwMap.servo.get("LRS");
        capHold = hwMap.servo.get("CBHS");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        collectionMotor.setDirection(DcMotor.Direction.REVERSE);
        flickerMotor.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialize sensors
        imu = new AdafruitIMU(opmode, "IMU");
        leftLightSensor = hwMap.lightSensor.get("LSL");
        rightLightSensor = hwMap.lightSensor.get("LSR");
        leftSharpIR = hwMap.analogInput.get("SIRL");
        rightSharpIR = hwMap.analogInput.get("SIRR");
/*        leftUltrasonic = hwMap.analogInput.get("USL");
        rightUltrasonic = hwMap.analogInput.get("USR");*/
        rightFrontColor = hwMap.colorSensor.get("RFCS");
        rightBackColor = hwMap.colorSensor.get("RBCS");
        leftFrontColor = hwMap.colorSensor.get("LFCS");
        leftBackColor = hwMap.colorSensor.get("LBCS");

        rightFrontColor.setI2cAddress(I2cAddr.create8bit(0x3c));
        rightBackColor.setI2cAddress(I2cAddr.create8bit(0x3e));
        leftFrontColor.setI2cAddress(I2cAddr.create8bit(0x40));
        leftBackColor.setI2cAddress(I2cAddr.create8bit(0x42));

        //Initialize sensors
        leftLightSensor.enableLed(true);
        rightLightSensor.enableLed(true);


        rightFrontColor.enableLed(false);
        rightBackColor.enableLed(false);
        leftFrontColor.enableLed(false);
        leftBackColor.enableLed(false);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        collectionMotor.setPower(0);
        flickerMotor.setPower(0);
        liftLeft.setPower(0);
        liftRight.setPower(0);

        // Set all servo home positions
        ballIndexer.setPosition(BIS_IDLE);
        leftButtonPushers.setPosition(LBPS_CENTER);
        rightButtonPushers.setPosition(RBPS_CENTER);
        frontPusher.setPosition(FPS_HOLD);
        liftRelease.setPosition(LRS_HOLD);
        capHold.setPosition(CBHS_IDLE);

        // Set all motors to run without encoders
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flickerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Sensor get values
    public double getLeftDistance() {
        return 12.08 * Math.pow(leftSharpIR.getVoltage(), -1.058);
        //double distUS = leftUltrasonic.getVoltage() / (leftUltrasonic.getMaxVoltage() / 1024);
        //return distIR > 23 ? distUS : distIR;
    }

    public double getRightDistance() {
        return 12.08 * Math.pow(rightSharpIR.getVoltage(), -1.058);
        //double distUS = rightUltrasonic.getVoltage() / (rightUltrasonic.getMaxVoltage() / 1024);
        //return distIR > 23 ? distUS : distIR;
    }

    public void tankDrive(double leftPower, double rightPower) {
        leftFrontMotor.setPower(leftPower);
        leftBackMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        rightBackMotor.setPower(rightPower);
    }

    public int leftBeaconStatus() {
        //0: RB; 1: BR; 2: RR; 3: BB; -1: Unknown
        boolean frontRed = leftFrontColor.red() > leftFrontColor.blue() && leftFrontColor.red() > leftFrontColor.green();
        boolean backRed = leftBackColor.red() > leftBackColor.blue() && leftBackColor.red() > leftBackColor.green();

        return (frontRed & !backRed ? 0 : (!frontRed & backRed ? 1 : (frontRed & backRed ? 2 : (!frontRed & !backRed ? 3 : -1))));
    }

    public int rightBeaconStatus() {
        //0: RB; 1: BR; 2: RR; 3: BB; -1: Unknown
        boolean frontRed = leftFrontColor.red() > leftFrontColor.blue() && leftFrontColor.red() > leftFrontColor.green();
        boolean backRed = leftBackColor.red() > leftBackColor.blue() && leftBackColor.red() > leftBackColor.green();

        return (frontRed & !backRed ? 0 : (!frontRed & backRed ? 1 : (frontRed & backRed ? 2 : 3)));
    }

    public void wallFollowLeft(double speed, double wallSetPoint, double Kp, double Ki, double Kd ) {
        double dist = getLeftDistance();
        double distError = dist - wallSetPoint;

        wallIntegral += distError;
        double wallDerivative = distError - wallPreError;

        wallPreError = distError;

        double output = (Kp * distError) + (Ki * wallIntegral) + (Kd * wallDerivative);

        if (speed < 0) {
            tankDrive(Range.clip(speed - output, -1, 0.5), Range.clip(speed + output, -1, 0.5));
        } else {
            tankDrive(Range.clip(speed + output, -0.5, 1), Range.clip(speed - output, -0.5, 1));
        }
    }
}