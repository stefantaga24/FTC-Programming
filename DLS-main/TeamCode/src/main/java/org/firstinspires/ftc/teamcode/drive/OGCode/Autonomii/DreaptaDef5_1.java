package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;

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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;


@Config
@Autonomous(group = "drive")

public class DreaptaDef5_1 extends LinearOpMode {
    enum STROBOT
    {
        START,
        FIRST_CYCLE,
        SECOND_CYCLE,
        THIRD_CYCLE,
        FOURTH_CYCLE,
        FIFTH_CYCLE,
        RETRACT,
        PARK,
        GET_FIRST_LIFT_DOWN,
        GO_TO_CYCLING_BABY,
        STOP_JOC,
        GET_SECOND_LIFT_DOWN,
        GET_SECOND_LIFT_UP,
        PRELOAD,
        GET_DOWN
    }
    public static double x_PLACE_CONE = 35 , y_PLACE_CONE = -3 , Angle_PLACE_CONE = 0;
    public static double x_CYCLING_POSITION = 36.5, y_CYCLING_POSITION = -19.5, Angle_CYCLING_POSITION = 12;
    public static double x_INTER = 35.5, y_INTER = -30 , Angle_PARK_INTER = 270;
    public static double x_PARK1 = 10, y_PARK1 = -25, Angle_PARK1 = 270;
    public static double x_PARK2 = 36.5, y_PARK2 = -25, Angle_PARK2 = 270;
    public static double x_PARK3 = 57, y_PARK3 = -25, Angle_PARK3 = 270;
    ElapsedTime asteapta = new ElapsedTime(), timerRetract = new ElapsedTime(), timerLift =new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        timeOutBaby = 1.5;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);



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
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
        Angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;

        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();

        autoController51.Cone_Stack_Level  =5;
        autoController51.AutoLiftStatus = LiftController.LiftStatus.MID;
        autoController51.LimitLift = 0.75;
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
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        Pose2d cyclingPosition = new Pose2d(x_CYCLING_POSITION,y_CYCLING_POSITION,Math.toRadians(Angle_CYCLING_POSITION));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_PLACE_CONE,y_PLACE_CONE,Math.toRadians(Angle_PLACE_CONE)))
                .build();
        TrajectorySequence GO_TO_CYCLING_POSITION = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineToLinearHeading(cyclingPosition)
                .build();
        TrajectorySequence PARK1 = drive.trajectorySequenceBuilder(cyclingPosition)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK1,y_PARK1,Math.toRadians(Angle_PARK1)))
                .build();
        TrajectorySequence PARK2 = drive.trajectorySequenceBuilder(cyclingPosition)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK2,y_PARK2,Math.toRadians(Angle_PARK2)))
                .build();
        TrajectorySequence PARK3 = drive.trajectorySequenceBuilder(cyclingPosition)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK3,y_PARK3,Math.toRadians(Angle_PARK3)))
                .build();
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        PipeLineDetector detector = new PipeLineDetector(270,155,320,185);
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
                    status = STROBOT.PRELOAD;
                    break;
                }
                case PRELOAD:
                {
                    if (!drive.isBusy())
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                        timerLift.reset();
                        status = STROBOT.GET_FIRST_LIFT_DOWN;
                    }
                    break;
                }
                case GET_FIRST_LIFT_DOWN:
                {
                    if (timerLift.seconds()>0.75)
                    {
                        timerLift.reset();
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        status = STROBOT.GO_TO_CYCLING_BABY;
                    }
                    break;
                }
                case GO_TO_CYCLING_BABY:
                {
                    if (timerLift.seconds()>1)
                    {
                        drive.followTrajectorySequenceAsync(GO_TO_CYCLING_POSITION);
                        status = STROBOT.GET_SECOND_LIFT_UP;
                    }
                    break;
                }
                case GET_SECOND_LIFT_UP:
                {
                    if (!drive.isBusy())
                    {
                        timerLift.reset();
                        liftController.CurrentStatus = LiftController.LiftStatus.MID;
                        status = STROBOT.GET_SECOND_LIFT_DOWN;
                    }
                    break;
                }
                case GET_SECOND_LIFT_DOWN:
                {
                    if (timerLift.seconds()>1)
                    {
                        timerLift.reset();
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        status = STROBOT.FIRST_CYCLE;
                    }
                    break;
                }
                case FIRST_CYCLE:
                {
                    if (timerLift.seconds()>0.5)
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.SECOND_CYCLE;
                    }
                    break;
                }
                case SECOND_CYCLE:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.THIRD_CYCLE;
                    }
                    break;
                }
                case THIRD_CYCLE:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.RETRACT;
                    }
                    break;
                }
                case FOURTH_CYCLE:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
                        autoController51.CurrentStatus = AutoController5_1.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.FIFTH_CYCLE;
                    }
                    break;
                }
                case FIFTH_CYCLE:
                {
                    if (autoController51.CurrentStatus == AutoController5_1.autoControllerStatus.NOTHING)
                    {
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
            motorColectareController.update(robot,ColectarePosition, 0.6, currentVoltage);
            liftController.update(robot,LiftPosition,sigurantaLiftController,currentVoltage);
            autoController51.update(sigurantaLiftController,robot,angle4BarController, turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);

            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}