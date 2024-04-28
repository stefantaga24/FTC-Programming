package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.hardware.Sensor;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;



import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class RobotMap {

    public Servo left4Bar = null;
    public Servo right4Bar = null;
    public Servo turnClaw = null;
    public DistanceSensor dsensor = null;
    public DigitalChannel coneguide = null;
    Servo closeClaw = null;
    Servo servoLift = null;
    Servo sigurantaLift = null;
    Servo angle4Bar = null;
    public DcMotorEx motorColectareStanga = null;
    public DcMotorEx motorColectareDreapta = null;
    public DcMotorEx dreaptaLift = null;
    DcMotorEx stangaLift = null;
    public static boolean USING_IMU = true;
    public static int xAI = 300,yAI = 150,xBI = 320,yBI = 180;

    /*private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private static double imuAngle = 0;
    private static double imuOffset = 0;*/


    public RobotMap(HardwareMap Init)
    {

        /*if (USING_IMU) {
            synchronized (imuLock) {
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
            }
        }*/

        left4Bar = Init.get(Servo.class, "left4Bar");
        right4Bar = Init.get(Servo.class,"right4Bar");

        turnClaw = Init.get(Servo.class, "turnClaw");
        closeClaw = Init.get(Servo.class, "closeClaw");
        servoLift = Init.get(Servo.class,"servoLift");
        sigurantaLift = Init.get(Servo.class,"sigurantaLift");
        angle4Bar = Init.get(Servo.class,"angle4Bar");
        dsensor = Init.get(DistanceSensor.class, "dsensor");
       coneguide = Init.get(DigitalChannel.class, "coneguide");
        coneguide.setMode(DigitalChannel.Mode.INPUT);

        motorColectareStanga = Init.get(DcMotorEx.class, "motorColectareStanga");
        motorColectareDreapta = Init.get(DcMotorEx.class, "motorColectareDreapta");
        stangaLift = Init.get(DcMotorEx.class, "stangaLift");
        dreaptaLift = Init.get(DcMotorEx.class, "dreaptaLift");


        right4Bar.setDirection(Servo.Direction.REVERSE);
        left4Bar.setDirection(Servo.Direction.REVERSE);

        stangaLift.setDirection(DcMotorSimple.Direction.REVERSE);

        dreaptaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorColectareStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorColectareStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorColectareStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        motorColectareDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorColectareDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorColectareDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorColectareDreapta.setDirection(DcMotorSimple.Direction.REVERSE);



    }

   /* public void reset() {
        try {
        } catch (Exception e) {}
        imuOffset = imu.getAngularOrientation().firstAngle;
    }
    public static double getAngle() {
        return imuAngle - imuOffset;
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (USING_IMU) {
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = imu.getAngularOrientation().firstAngle;
                    }
                }
            });
            imuThread.start();
        }
    }*/

}