package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.timeOutBaby;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoLeftSouthHighJunction5_1;
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
@Autonomous(group = "WORLDS")

public class StangaHaz2_What extends LinearOpMode {
    enum STROBOT
    {
        START,
        GET_LIFT_DOWN,
        PARK,
        STOP_JOC,
        GET_LIFT_UP,
        TURN_TO_COLLECT,
        COLLECT,
        GO_TO_SCORING_POSITION,
        GO_TO_COLLECTING_POSITION,
    }
    public static double x_SLEEVE = -35 , y_SLEEVE = 3 ;
    public static double x_PLACE_PRELOAD = -37, y_PLACE_PRELOAD= -9, Angle_PLACE_PRELOAD = 225;
    public static double x_CYCLING_POSITION = -10, y_CYCLING_POSITION = -61;
    public static double x_COLLECT_POSITION = -18.5, y_COLLECT_POSITION = -11.5, Angle_COLLECT_POSITION = 180.25;
    public static double x_PLACE_SOUTH_HIGH = -13, y_PLACE_SOUTH_HIGH = -17.5, Angle_PLACE_SOUTH_HIGH = 155;
    public static double x_PARK1 = -60, y_PARK1 = -10, Angle_PARK1 = 180;
    public static double x_PARK2 = -35, y_PARK2 = -10, Angle_PARK2 = 180;
    public static double x_PARK3 = -12.5, y_PARK3 = -10, Angle_PARK3 = 180;
    TrajectoryVelocityConstraint CONSTRAINTO = getVelocityConstraint(40, 7.6, TRACK_WIDTH);

    ElapsedTime asteapta = new ElapsedTime(), timerRetract = new ElapsedTime(), timerLift =new ElapsedTime() , timeCollect = new ElapsedTime();


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
        timeOutBaby = 0.35;
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
        AutoLeftSouthHighJunction5_1 autoControllerTurn51 = new AutoLeftSouthHighJunction5_1();
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
        autoControllerTurn51.AutoLiftStatus = LiftController.LiftStatus.HIGH_SOUTH;
        autoControllerTurn51.LimitLift = 0.75;
        motorColectareController.NrConAuto = 5;
        robot.turnClaw.setPosition(TurnClawController.pozTurnClaw_COLLECT);

        angle4BarController.update(robot);
        closeClawController.update(robot);
        turnClawController.update(robot);
        servo4BarController.update(robot);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 0.6, currentVoltage);
        liftController.update(robot,0,sigurantaLiftController,currentVoltage);
        robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
        sigurantaLiftController.update(robot);
        int nr=0;
        ElapsedTime timeStart = new ElapsedTime() , timeTurnPlace = new ElapsedTime(), timerGOPLACE = new ElapsedTime();
        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(270));
        Pose2d PLACE_SOUTH_HIGH = new Pose2d(x_PLACE_SOUTH_HIGH,y_PLACE_SOUTH_HIGH,Math.toRadians(Angle_PLACE_SOUTH_HIGH));
        Pose2d COLLECT_POSITION_5 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION));
        Pose2d COLLECT_POSITION_4 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION-0.2));
        Pose2d COLLECT_POSITION_3 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION-0.2));
        Pose2d COLLECT_POSITION_2 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION-0.2));
        Pose2d COLLECT_POSITION_1 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION+0.5,Math.toRadians(Angle_COLLECT_POSITION-0.2));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(CONSTRAINTO)
                .lineTo(new Vector2d(x_SLEEVE,y_SLEEVE))
                .lineToLinearHeading(new Pose2d(x_PLACE_PRELOAD,y_PLACE_PRELOAD,Math.toRadians(Angle_PLACE_PRELOAD)))
                .build();
        TrajectorySequence GO_TO_PLACE_POSITION = drive.trajectorySequenceBuilder(COLLECT_POSITION_1)
                .lineToLinearHeading(PLACE_SOUTH_HIGH)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_1 = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineToLinearHeading(COLLECT_POSITION_1)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_2 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .lineToLinearHeading(COLLECT_POSITION_2)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_3 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .lineToLinearHeading(COLLECT_POSITION_3)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_4 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .lineToLinearHeading(COLLECT_POSITION_4)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_5 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .lineToLinearHeading(COLLECT_POSITION_5)
                .build();
        TrajectorySequence PARK1 = drive.trajectorySequenceBuilder(GO_TO_PLACE_POSITION.end())
                .lineToLinearHeading(new Pose2d(x_PARK1,y_PARK1,Math.toRadians(Angle_PARK1)))
                .build();
        TrajectorySequence PARK2 = drive.trajectorySequenceBuilder(GO_TO_PLACE_POSITION.end())
                .lineToLinearHeading(new Pose2d(x_PARK2,y_PARK2,Math.toRadians(Angle_PARK2)))
                .build();
        TrajectorySequence PARK3 = drive.trajectorySequenceBuilder(GO_TO_PLACE_POSITION.end())
                .lineToLinearHeading(new Pose2d(x_PARK3,y_PARK3,Math.toRadians(Angle_PARK3)))
                .build();
        double WhenToStartCollecting = 1;
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
            switch (status) {
                case START: {
                    drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                    status = STROBOT.GET_LIFT_UP;
                    break;
                }
                case GET_LIFT_UP: {
                    if (nr==0)
                    {
                        if (!drive.isBusy()) {
                            liftController.CurrentStatus = LiftController.LiftStatus.HIGH_SOUTH;
                            timerLift.reset();
                            status = STROBOT.GET_LIFT_DOWN;
                        }
                    }
                    else
                    {
                        if (timerGOPLACE.seconds()>1.1) {
                            liftController.CurrentStatus = LiftController.LiftStatus.HIGH_SOUTH;
                            timerLift.reset();
                            status = STROBOT.GET_LIFT_DOWN;
                        }
                    }
                    break;
                }
                case GET_LIFT_DOWN: {
                        if (timerLift.seconds() > autoControllerTurn51.LimitLift) {
                            timerLift.reset();
                            liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                            status = STROBOT.GO_TO_COLLECTING_POSITION;
                        }
                    break;
                }
                case COLLECT:
                {
                    if (nr==5)
                    {
                        status = STROBOT.PARK;
                    }
                    else
                    {
                        if (timeCollect.seconds()>WhenToStartCollecting)
                        {
                            nr++;
                            autoControllerTurn51.CurrentStatus = AutoLeftSouthHighJunction5_1.autoControllerSouthHigh.STACK_LEVEL;
                            status = STROBOT.GO_TO_SCORING_POSITION;
                        }
                    }
                    break;
                }
                case GO_TO_SCORING_POSITION:
                {
                    if (autoControllerTurn51.CurrentStatus  == AutoLeftSouthHighJunction5_1.autoControllerSouthHigh.PLACE_CONE)
                    {
                        drive.followTrajectorySequenceAsync(GO_TO_PLACE_POSITION);
                        timerGOPLACE.reset();
                        status = STROBOT.GET_LIFT_UP;
                    }
                    break;
                }
                case GO_TO_COLLECTING_POSITION:
                {
                    if (nr==5)
                    {
                        status = STROBOT.PARK;
                    }
                    else
                    {
                        motorColectareController.NrConAuto = motorColectareController.NrConAuto - 1;
                        if (nr==0)
                        {
                            WhenToStartCollecting = 3;
                        }
                        else
                        {
                            WhenToStartCollecting = 1;
                        }
                        switch (4-nr)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_5);
                                break;
                            }
                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_4);
                                break;
                            }
                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_3);
                                break;
                            }
                            case 3:
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_2);
                                break;
                            }
                            case 4:
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_1);
                                break;
                            }
                        }
                        timeCollect.reset();
                        status = STROBOT.COLLECT;
                    }
                    break;
                }
                case PARK:
                {
                    if (!drive.isBusy())
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