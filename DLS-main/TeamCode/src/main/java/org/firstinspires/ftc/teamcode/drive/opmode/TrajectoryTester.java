package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TrajectoryTester extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double x_CYCLING_POSITION = 36, y_CYCLING_POSITION = -20, Angle_CYCLING_POSITION = 10;

    public static double x_CYCLING_POSITION_LEFT = -33, y_CYCLING_POSITION_LEFT = -3, Angle_CYCLING_POSITION_LEFT = 192;
    public static double x_SSH_LEFT = -10 ,y_SSH_LEFT = -10 , Angle_SSH_LEFT = 180, Tanget_Angle_SSH_LEFT = 180;
    public static double x_LINETO_LEFT = 29 , y_LINETO_LEFT = -10;
    public static double x_LLH_LEFT = 33 , y_LLH_LEFT = -3, Angle_LLH_LEFT = 345;
    public static double xlt = 19, ylt = -59, anglespline = 10, xstc = 15, ystc=-50, xsts = 15, ysts = -35, xlt1 = 15, ylt1 = -17, xstc2 = 16, ystc2 = -16;
    public static double xltl = 35, yltl=-10, angleltlt=15, xl=15, yl=-20;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        Pose2d StartPositionRight = new Pose2d(35,-60,Math.toRadians(270));
        Pose2d StartPositionLeft = new Pose2d(-35,-60,Math.toRadians(270));
        drive.setPoseEstimate(StartPositionRight);
        Servo4BarController servo4BarController = new Servo4BarController();
        MotorColectareController motorColectareController = new MotorColectareController();
        TrajectorySequence AutonomiaDreapta = drive.trajectorySequenceBuilder(StartPositionRight)
                .strafeRight(1)
                .splineToSplineHeading(new Pose2d(10,-40,Math.toRadians(180)),Math.toRadians(180))
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(AutonomiaDreapta);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
