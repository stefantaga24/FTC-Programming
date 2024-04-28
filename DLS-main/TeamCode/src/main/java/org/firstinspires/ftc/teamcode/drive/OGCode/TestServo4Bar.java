package org.firstinspires.ftc.teamcode.drive.OGCode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="TestServo4Bar", group="Testers")

public class TestServo4Bar extends  LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static int salut=0;
    double poz1 = 0 ,poz2 = 1;
    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }
    @Override
    public void runOpMode() {

        RobotMap robot=new RobotMap(hardwareMap);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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

            if (!previousGamepad1.circle && currentGamepad1.circle)
            {
                robot.left4Bar.setPosition(0.5);
                robot.right4Bar.setPosition(0.5);
            }
            if (!previousGamepad1.triangle && currentGamepad1.triangle)
            {
                robot.left4Bar.setPosition(0.25);
                robot.right4Bar.setPosition(0.25);
            }
            if (!previousGamepad1.cross && currentGamepad1.cross)
            {
                robot.left4Bar.setPosition(0.85);
                robot.right4Bar.setPosition(0.85);
            }
            if (!previousGamepad1.dpad_up && currentGamepad1.dpad_up)
            {
                robot.angle4Bar.setPosition(Angle4BarController.pozVertical);
            }
            telemetry.addData("left4Bar", robot.left4Bar.getPosition());
            telemetry.addData("angle4Bar",robot.angle4Bar.getPosition());
            telemetry.update();
        }
    }
}
