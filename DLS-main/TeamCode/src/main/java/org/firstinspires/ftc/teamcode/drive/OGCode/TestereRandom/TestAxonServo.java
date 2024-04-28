package org.firstinspires.ftc.teamcode.drive.OGCode.TestereRandom;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="TestAxonServo", group="Testers")

public class TestAxonServo extends  LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public double pos1 = 0 ,pos2 =1;
    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }
    @Override
    public void runOpMode() {


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Servo servo1 = hardwareMap.get(Servo.class,"servo1Axon");

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
            if (!previousGamepad1.a && currentGamepad1.a)
            {
                servo1.setPosition(0);
            }
            if (!previousGamepad1.b && currentGamepad1.b)
            {
                servo1.setPosition(1);
            }
            telemetry.update();
        }
    }
}
