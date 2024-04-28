package org.firstinspires.ftc.teamcode.drive.OGCode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="TestDrive", group="Testers")

public class TestDrive extends  LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static int salut=0;
    double pozInit4Bar = 0, pozInter4Bar= 0.4, pozPlace4Bar = 0.7;
    double pozCloseClaw=0.8, pozOpenClaw=0.2;
    double kp =0, ki=0, kd=0;
    boolean isDown = true, isClosed=false, isTurned = false, isExtended = false;
    double  PrecisionDenominator=1, PrecisionDenominator2=1.25;
    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double  lim, boolean StrafesOn , double LeftTrigger, double RightTrigger)
    {
        double y = gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x*1.1;
        if (StrafesOn == false)
        {
            x=0;
        }
        double rx = gamepad1.left_stick_x*1 - LeftTrigger + RightTrigger;

        rx/=PrecisionDenominator2;
        x/=PrecisionDenominator;
        y/=PrecisionDenominator;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    @Override
    public void runOpMode() {

        RobotMap robot=new RobotMap(hardwareMap);
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        imu.initialize(parameters);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lim = 1 ; /// limita vitezei la sasiu
        String typeOfDrive = "RobotCentric";
        DcMotor rightFront = null;
        DcMotor rightBack = null;
        DcMotor leftFront = null;
        DcMotor leftBack = null;
        rightFront = hardwareMap.get(DcMotor.class,"leftFront");
        leftFront = hardwareMap.get(DcMotor.class,"rightFront");
        rightBack = hardwareMap.get(DcMotor.class,"leftBack");
        leftBack = hardwareMap.get(DcMotor.class,"rightBack");


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            /// DRIVE
            robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim,true , 0,0);
            telemetry.addData("motorColectarePositionStanga",robot.motorColectareStanga.getCurrentPosition());
            telemetry.addData("motorColectarePower",robot.motorColectareStanga.getPower());
            telemetry.update();
        }
    }
}
