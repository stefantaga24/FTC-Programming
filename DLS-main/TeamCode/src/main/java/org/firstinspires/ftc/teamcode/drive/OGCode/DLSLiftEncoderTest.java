package org.firstinspires.ftc.teamcode.drive.OGCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Testers")
public class DLSLiftEncoderTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double Kp = 0.0065;// 0.006
    public static double Ki = 0.0055; // 0.0045
    public static double Kd = 0;
    public static double Kg = 0;
    public static double maxSpeed = 1;
    public static double RetractedPosition = 0 , ExtendedPosition = 700;
    int TargetLift = 0;
    ElapsedTime timerPID = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {
        List <LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ElapsedTime changePositions = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap robot = new RobotMap(hardwareMap);
        SimplePIDController hello = new SimplePIDController(Kp,Ki,Kd);
        waitForStart();

        if (isStopRequested()) return;


        telemetry.update();
        hello.targetValue = RetractedPosition;

        while (!isStopRequested() && opModeIsActive())
        {
            int DreaptaLiftPosition = robot.dreaptaLift.getCurrentPosition();
            double powerDreaptaLift = hello.update(DreaptaLiftPosition) + Kg;
            powerDreaptaLift = Math.max(-1,Math.min(powerDreaptaLift,1));
            robot.dreaptaLift.setPower(powerDreaptaLift);
            robot.stangaLift.setPower(powerDreaptaLift);
            if (changePositions.seconds()>3)
            {
                if (hello.targetValue == RetractedPosition )
                {
                    hello.targetValue = ExtendedPosition;
                    SigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                }
                else
                {
                    hello.targetValue = RetractedPosition;
                }
                changePositions.reset();
            }

            telemetry.addData("ColectareEncoder", DreaptaLiftPosition);
            telemetry.addData("powerColectare", powerDreaptaLift);
            telemetry.addData("TargetLift",hello.targetValue);
            telemetry.addData("siguranta", SigurantaLiftController.CurrentStatus);
            telemetry.addData("Error", hello.measuredError(DreaptaLiftPosition));
            if (Kp!=hello.p || Kd!=hello.d || Ki!=hello.i || maxSpeed !=hello.maxOutput )
            {
                hello.p = Kp;
                hello.d = Kd;
                hello.i = Ki;
                hello.maxOutput = maxSpeed;
            }

            telemetry.update();
        }
    }
}