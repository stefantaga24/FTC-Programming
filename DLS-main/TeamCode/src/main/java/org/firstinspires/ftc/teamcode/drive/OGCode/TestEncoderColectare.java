package org.firstinspires.ftc.teamcode.drive.OGCode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp(name="TestEncoderColectare", group="Testers")

public class TestEncoderColectare extends  LinearOpMode {
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
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            /// DRIVE
            robot.motorColectareStanga.setPower(gamepad1.right_stick_y);
            robot.motorColectareDreapta.setPower(gamepad1.right_stick_y);
            telemetry.addData("motorColectarePositionStanga",robot.motorColectareStanga.getCurrentPosition());
            telemetry.addData("motorColectarePower",robot.motorColectareStanga.getPower());
            telemetry.addData("curent", robot.motorColectareDreapta.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
