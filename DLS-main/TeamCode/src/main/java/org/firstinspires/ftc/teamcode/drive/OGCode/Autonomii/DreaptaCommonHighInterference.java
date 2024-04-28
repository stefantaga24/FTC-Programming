package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;


import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_CommonHighInterference.autoControllerSouthHigh.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.timeOutBaby;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_CommonHighInterference;
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
@Autonomous(group = "drive")

public class DreaptaCommonHighInterference extends LinearOpMode {
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
    public static double x_CYCLING_POSITION = 36, y_CYCLING_POSITION = -12.5, Angle_CYCLING_POSITION = 315;
    public static double x_COLLECT_POSITION = 45, y_COLLECT_POSITION = -12, Angle_COLLECT_POSITION = 357.5;
    public static double x_PLACE_SOUTH_HIGH = 34, y_PLACE_SOUTH_HIGH = -13, Angle_PLACE_SOUTH_HIGH = 310;
    public static double x_PARK1 = 13.5, y_PARK1 = -15, Angle_PARK1 = 0;
    public static double x_PARK2 = 35, y_PARK2 = -15, Angle_PARK2 = 0;
    public static double x_PARK3 = 60, y_PARK3 = -15, Angle_PARK3 = 0;
    public static double Angle_TURN_COLLECT = 40;
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
        AutoController5_1_CommonHighInterference autoControllerTurn51 = new AutoController5_1_CommonHighInterference();
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
        autoControllerTurn51.LimitLift = 0.6;
        motorColectareController.NrConAuto = 5;
        robot.turnClaw.setPosition(TurnClawController.pozTurnClaw_COLLECT);

        angle4BarController.update(robot);
        closeClawController.update(robot);
        turnClawController.update(robot);
        servo4BarController.update(robot);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 0.6,currentVoltage);
        liftController.update(robot,0,sigurantaLiftController,currentVoltage);
        robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
        sigurantaLiftController.update(robot);
        int nr=0;
        TrajectoryVelocityConstraint VELLLH = getVelocityConstraint(50, 4, 13.58);
        ElapsedTime timeStart = new ElapsedTime() , timeTurnPlace = new ElapsedTime();
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        Pose2d PLACE_SOUTH_HIGH = new Pose2d(x_PLACE_SOUTH_HIGH,y_PLACE_SOUTH_HIGH,Math.toRadians(Angle_PLACE_SOUTH_HIGH));
        Pose2d COLLECT_POSITION = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        Pose2d COLLECT_POSITION_5 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION));
        Pose2d COLLECT_POSITION_4 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION-1,Math.toRadians(Angle_COLLECT_POSITION+0.15));
        Pose2d COLLECT_POSITION_3 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION-1.2,Math.toRadians(Angle_COLLECT_POSITION+0.3));
        Pose2d COLLECT_POSITION_2 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION-1.4,Math.toRadians(Angle_COLLECT_POSITION+0.45));
        Pose2d COLLECT_POSITION_1 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION-1.6,Math.toRadians(Angle_COLLECT_POSITION+0.6));
        TrajectorySequence GO_TO_COLLECTING_POSITION_5 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(COLLECT_POSITION_5)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_4 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(COLLECT_POSITION_4)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_3 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(COLLECT_POSITION_3)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_2 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(COLLECT_POSITION_2)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION_1 = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(COLLECT_POSITION_1)
                .build();
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_CYCLING_POSITION,y_CYCLING_POSITION,Math.toRadians(Angle_CYCLING_POSITION)))
                .build();
        TrajectorySequence TURN_TO_COLLECT = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .turn(Math.toRadians(Angle_TURN_COLLECT))
                .build();
        TrajectorySequence GO_TO_PLACE_POSITION = drive.trajectorySequenceBuilder(COLLECT_POSITION)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(PLACE_SOUTH_HIGH)
                .build();
        TrajectorySequence GO_TO_COLLECTING_POSITION = drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(COLLECT_POSITION)
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
                    if (!drive.isBusy()) {
                        liftController.CurrentStatus = LiftController.LiftStatus.HIGH_SOUTH;
                        timerLift.reset();
                        status = STROBOT.GET_LIFT_DOWN;
                    }
                    break;
                }
                case GET_LIFT_DOWN: {
                    if (nr==0)
                    {
                        if (timerLift.seconds() > 0.85) {
                            timerLift.reset();
                            liftController.CurrentStatus = LiftController.LiftStatus.BASE_BAZAVAN;
                            status = STROBOT.TURN_TO_COLLECT;
                        }
                    }
                    else
                    {
                        if (timerLift.seconds() > 0.85) {
                            timerLift.reset();
                            liftController.CurrentStatus = LiftController.LiftStatus.BASE_BAZAVAN;
                            status = STROBOT.GO_TO_COLLECTING_POSITION;
                        }
                    }
                    break;
                }
                case TURN_TO_COLLECT:
                {
                    if (timerLift.seconds() > 0.45)
                    {
                        timeTurnPlace.reset();
                        drive.followTrajectorySequenceAsync(TURN_TO_COLLECT);
                        status = STROBOT.COLLECT;
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
                        if (nr==0) {
                            if (timeTurnPlace.seconds()>0.6)
                            {
                                nr++;
                                autoControllerTurn51.CurrentStatus = AutoController5_1_CommonHighInterference.autoControllerSouthHigh.STACK_LEVEL;
                                status = STROBOT.GO_TO_SCORING_POSITION;
                            }
                        }
                        else
                        {
                            if (timeCollect.seconds()>1)
                            {
                                motorColectareController.NrConAuto = motorColectareController.NrConAuto-1;
                                nr++;
                                autoControllerTurn51.CurrentStatus = AutoController5_1_CommonHighInterference.autoControllerSouthHigh.STACK_LEVEL;
                                status = STROBOT.GO_TO_SCORING_POSITION;
                            }
                        }
                    }
                    break;
                }
                case GO_TO_SCORING_POSITION:
                {
                    if (autoControllerTurn51.CurrentStatus == PLACE_CONE)
                    {
                        drive.followTrajectorySequenceAsync(GO_TO_PLACE_POSITION);
                        status = STROBOT.GET_LIFT_UP;
                    }
                    break;
                }
                case GO_TO_COLLECTING_POSITION:
                {
                    if (timerLift.seconds()>0.1)
                    {
                        if (nr==5)
                        {
                            status = STROBOT.PARK;
                        }
                        else
                        {
                            if (nr==0) {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_5);
                            }
                            else
                            if (nr==1)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_4);
                            }
                            else
                            if (nr==2)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_3);
                            }
                            else
                            if (nr==3)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_2);
                            }
                            else
                            if (nr==4)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_COLLECTING_POSITION_1);
                            }
                            timeCollect.reset();
                            status = STROBOT.COLLECT;
                        }
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