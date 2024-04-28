package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.timeOutBaby;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoTurnJunction5_1;
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

import java.util.ArrayList;
import java.util.List;


@Config
@Autonomous(group = "drive")

public class DreaptaHTurnJunction5_1 extends LinearOpMode {
    enum STROBOT
    {
        START,
        GET_LIFT_DOWN,
        PARK,
        STOP_JOC,
        GET_LIFT_UP,
        GO_CYCLE,
        TURN_TO_PLACE,
        START_CYCLE,
    }
    public static double x_CYCLING_POSITION = 35.5, y_CYCLING_POSITION = -12 , Angle_CYCLING_POSITION = 315;
    public static double x_PARK1 = 10, y_PARK1 = -17, Angle_PARK1 = 90;
    public static double x_PARK2 = 35, y_PARK2 = -17, Angle_PARK2 = 90;
    public static double x_PARK3 = 55, y_PARK3 = -17, Angle_PARK3 = 90;
    public static double Angle_TURN_COLLECT = 90 ,Angle_TURN_TO_PLACE = -90;
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
        timeOutBaby = 0.25;
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
        AutoTurnJunction5_1 autoControllerTurn51 = new AutoTurnJunction5_1();
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



        autoControllerTurn51.Cone_Stack_Level  =5;
        autoControllerTurn51.AutoLiftStatus = LiftController.LiftStatus.HIGH;
        autoControllerTurn51.LimitLift = 0.85;


        angle4BarController.update(robot);
        closeClawController.update(robot);
        turnClawController.update(robot);
        servo4BarController.update(robot);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 0.6 , currentVoltage);
        liftController.update(robot,0,sigurantaLiftController,currentVoltage);
        robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
        sigurantaLiftController.update(robot);
        int nr=0;
        ElapsedTime timeStart = new ElapsedTime() , timeTurnPlace = new ElapsedTime();
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(x_CYCLING_POSITION,y_CYCLING_POSITION))
                .lineTo(new Vector2d(x_CYCLING_POSITION-5,y_CYCLING_POSITION))
                .build();
        TrajectorySequence TURN_TO_COLLECT = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .turn(Math.toRadians(Angle_TURN_COLLECT))
                .build();
        TrajectorySequence TURN_TO_PLACE = drive.trajectorySequenceBuilder(TURN_TO_COLLECT.end())
                .turn(Math.toRadians(Angle_TURN_TO_PLACE))
                .build();
        TrajectorySequence PARK1 = drive.trajectorySequenceBuilder(TURN_TO_PLACE.end())
                .lineToLinearHeading(new Pose2d(x_PARK1,y_PARK1,Math.toRadians(Angle_PARK1)))
                .build();
        TrajectorySequence PARK2 = drive.trajectorySequenceBuilder(TURN_TO_PLACE.end())
                .lineToLinearHeading(new Pose2d(x_PARK2,y_PARK2,Math.toRadians(Angle_PARK2)))
                .build();
        TrajectorySequence PARK3 = drive.trajectorySequenceBuilder(TURN_TO_PLACE.end())
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
        while (opModeIsActive() && !isStopRequested())
        {
            int ColectarePosition = robot.motorColectareStanga.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista
            switch (status)
            {
                case START:
                {
                    drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                    status = STROBOT.GET_LIFT_UP;
                    break;
                }
                case GET_LIFT_UP:
                {
                    if (nr==0)
                    {
                        if (!drive.isBusy())
                        {
                            liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                            timerLift.reset();
                            status = STROBOT.GET_LIFT_DOWN;
                        }
                    }
                    else
                    {
                        if (timeTurnPlace.seconds()>0.2)
                        {
                            liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                            timerLift.reset();
                            status = STROBOT.GET_LIFT_DOWN;
                        }
                    }
                    break;
                }
                case GET_LIFT_DOWN:
                {
                    if (timerLift.seconds()>0.85)
                    {
                        timerLift.reset();
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        status = STROBOT.GO_CYCLE;
                    }
                    break;
                }
                case GO_CYCLE:
                {
                    if (timerLift.seconds()>0.45)
                    {
                        drive.followTrajectorySequenceAsync(TURN_TO_COLLECT);
                        nr++;
                        if (nr==6)
                        {
                            status = STROBOT.PARK;
                        }
                        else
                        {
                            timeStart.reset();
                            status = STROBOT.START_CYCLE;
                        }
                    }
                    break;
                }
                case START_CYCLE:
                {
                    if (timeStart.seconds()>0.35)
                    {
                        autoControllerTurn51.CurrentStatus = AutoTurnJunction5_1.autoControllerTurnStatus.STACK_LEVEL;
                        status = STROBOT.TURN_TO_PLACE;
                    }
                    break;
                }
                case TURN_TO_PLACE:
                {
                    if (autoControllerTurn51.CurrentStatus == AutoTurnJunction5_1.autoControllerTurnStatus.NOTHING)
                    {
                        drive.followTrajectorySequenceAsync(TURN_TO_PLACE);
                        timeTurnPlace.reset();
                        status = STROBOT.GET_LIFT_UP;
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
            motorColectareController.update(robot,ColectarePosition, 1, currentVoltage);
            liftController.update(robot,LiftPosition,sigurantaLiftController,currentVoltage);
            autoControllerTurn51.update(sigurantaLiftController,robot,angle4BarController, turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);

            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}