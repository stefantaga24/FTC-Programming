package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;

import static org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii.DreaptaMCyclingAutonomous5_1.Angle_PARK_INTER;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii.DreaptaMCyclingAutonomous5_1.x_INTER;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii.DreaptaMCyclingAutonomous5_1.y_INTER;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.timeOutBaby;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1;
import org.firstinspires.ftc.teamcode.drive.OGCode.BiggerController;
import org.firstinspires.ftc.teamcode.drive.OGCode.CloseClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.PipeLineDetector;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.SigurantaLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;


@Config
@Autonomous(group = "Worlds")

public class DreaptaHaz extends LinearOpMode {
    enum STROBOT
    {
        START,
        FIRST_CYCLE,
        LIFT_UP,
        SECOND_CYCLE,
        THIRD_CYCLE,
        FOURTH_CYCLE,
        FIFTH_CYCLE,
        RETRACT,
        PARK,
        STOP_JOC,
        PRELOAD,
        GET_DOWN
    }
    public static double x_CYCLING_POSITION = 37, y_CYCLING_POSITION = -19, Angle_CYCLING_POSITION = 13;
    public static double x_PLACE_PRELOAD = 37, y_PLACE_PRELOAD = 3, Angle_PLACE_PRELOAD = 20;
    public static double x_PARK1 = 10, y_PARK1 = -35, Angle_PARK1 = 0;
    public static double x_PARK2 = 35, y_PARK2 = -35, Angle_PARK2 = 0;
    public static double x_PARK3 = 60, y_PARK3 = -35, Angle_PARK3 = 0;
    ElapsedTime asteapta = new ElapsedTime(), timerRetract = new ElapsedTime(), timerLift =new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() throws InterruptedException {
        timeOutBaby = 0.5;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Servo4BarController servo4BarController = new Servo4BarController();
        MotorColectareController motorColectareController = new MotorColectareController();
        CloseClawController closeClawController = new CloseClawController();
        TurnClawController turnClawController = new TurnClawController();
        RobotController robotController = new RobotController();
        BiggerController biggerController = new BiggerController();
        LiftController liftController = new LiftController();
        AutoController5_1 autoController51 = new AutoController5_1();
        SigurantaLiftController sigurantaLiftController = new SigurantaLiftController();
        Angle4BarController angle4BarController = new Angle4BarController();
        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
        robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;


        motorColectareController.NrConAuto = 5;
        autoController51.Cone_Stack_Level  =5;
        autoController51.AutoLiftStatus = LiftController.LiftStatus.MID;
        autoController51.LimitLift = 0.65;
        robot.turnClaw.setPosition(TurnClawController.pozTurnClaw_COLLECT);
        angle4BarController.update(robot);
        closeClawController.update(robot);
        turnClawController.update(robot);
        servo4BarController.update(robot);
        motorColectareController.update(robot,0, 0.6, currentVoltage);
        liftController.update(robot,0,sigurantaLiftController,currentVoltage);
        robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
        sigurantaLiftController.update(robot);
        int nr=0;
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        Pose2d Cycling_Position = new Pose2d(x_CYCLING_POSITION,y_CYCLING_POSITION,Math.toRadians(Angle_CYCLING_POSITION));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_PLACE_PRELOAD,y_PLACE_PRELOAD,Math.toRadians(Angle_PLACE_PRELOAD)))
                .build();
        TrajectorySequence PARK1 = drive.trajectorySequenceBuilder(Cycling_Position)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK1,y_PARK1,Math.toRadians(Angle_PARK1)))
                .build();
        TrajectorySequence PARK2 = drive.trajectorySequenceBuilder(Cycling_Position)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))

                .lineToLinearHeading(new Pose2d(x_PARK2,y_PARK2,Math.toRadians(Angle_PARK2)))
                .build();
        TrajectorySequence PARK3 = drive.trajectorySequenceBuilder(Cycling_Position)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK3,y_PARK3,Math.toRadians(Angle_PARK3)))
                .build();
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        PipeLineDetector detector = new PipeLineDetector(robot.xAI,robot.yAI,robot.xBI,robot.yBI);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                }
        );
        PipeLineDetector.Status Case = PipeLineDetector.Status.ALBASTRU3;
        while (!isStarted()&&!isStopRequested())
        {
            Case = detector.caz;
            telemetry.addData("Caz", detector.caz);
            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        camera.closeCameraDevice();
        ElapsedTime timerStart = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested())
        {
            int ColectarePosition = robot.motorColectareStanga.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista

            switch (status)
            {
                case START:
                {
                    if (timerStart.seconds()>1.5)
                    {
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                        status = STROBOT.PRELOAD;
                    }
                    break;
                }
                case PRELOAD:
                {
                    if (!drive.isBusy())
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                        timerLift.reset();
                        status = STROBOT.LIFT_UP;
                    }
                    break;
                }
                case LIFT_UP:
                {
                    if (timerLift.seconds()>autoController51.LimitLift)
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        TrajectorySequence GO_TO_CYCLING = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(Cycling_Position)
                                .build();
                        drive.followTrajectorySequenceAsync(GO_TO_CYCLING);
                        status = STROBOT.FIRST_CYCLE;
                    }
                    break;
                }
                case FIRST_CYCLE:
                {
                    if (!drive.isBusy())
                    {
                        motorColectareController.NrConAuto = 5;
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.SECOND_CYCLE;
                    }
                    break;
                }
                case SECOND_CYCLE:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        motorColectareController.NrConAuto = 4;
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.THIRD_CYCLE;
                    }
                    break;
                }
                case THIRD_CYCLE:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        motorColectareController.NrConAuto = 3;
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.FOURTH_CYCLE;
                    }
                    break;
                }
                case FOURTH_CYCLE:
                {

                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        motorColectareController.NrConAuto = 2;
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.FIFTH_CYCLE;
                    }
                    break;
                }
                case FIFTH_CYCLE:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        motorColectareController.NrConAuto = 1;
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.RETRACT;
                    }
                    break;
                }
                case RETRACT:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                        timerRetract.reset();
                        status = STROBOT.PARK;
                    }
                    break;
                }
                case PARK:
                {
                    if (timerRetract.seconds()>0.3)
                    {
                        if (Case == PipeLineDetector.Status.VERDE1)
                        {
                            drive.followTrajectorySequenceAsync(PARK1);
                        }
                        else
                        if (Case == PipeLineDetector.Status.ROZ2)
                        {
                            drive.followTrajectorySequenceAsync(PARK2);
                        }
                        else
                        {
                            drive.followTrajectorySequenceAsync(PARK3);
                        }
                        status = STROBOT.STOP_JOC;
                    }
                    break;
                }
            }
            biggerController.update(robotController,closeClawController,motorColectareController);
            robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
            closeClawController.update(robot);
            angle4BarController.update(robot);
            turnClawController.update(robot);
            servo4BarController.update(robot);
            sigurantaLiftController.update(robot);
            motorColectareController.update(robot,ColectarePosition, 1,currentVoltage);
            liftController.update(robot,LiftPosition,sigurantaLiftController,currentVoltage);
            autoController51.update(sigurantaLiftController,robot,angle4BarController, turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);

            drive.update();
            //telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            //telemetry.addData("SpeedLift", liftController.CurrentSpeed);
           // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}