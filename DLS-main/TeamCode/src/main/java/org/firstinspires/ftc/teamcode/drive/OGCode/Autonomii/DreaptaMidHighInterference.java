package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.NO_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.PLACE_THE_CONE;
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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID;
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
@Autonomous(group = "MTI")

public class DreaptaMidHighInterference extends LinearOpMode {
    enum STROBOT
    {
        PULA_MEA,
        START,
        GET_LIFT_DOWN,
        PARK,
        INTER_GET_LIFT_UP,
        STOP_JOC,
        GET_LIFT_UP,
        TURN_TO_COLLECT,
        COLLECT,
        GO_TO_SCORING_POSITION,
        GO_TO_COLLECTING_POSITION,
        Slide1gen,
        Slide2gen,
        GET_LIFT_DOWN_INTF,
        VOLTAGE_CHECK,
        GET_LIFT_UP_INTF_VOLTAGE,
        GET_LIFT_DOWN_FOR_PRELOAD_ON_SOUTH,
        GO_TO_SCORING_POSITION_FOR_PRELOAD,
        GET_LIFT_UP_PRELOAD_ON_SOUTH,
        STUCK_CONE,
    }
    public static double x_COLLECT_POSITION = 37, y_COLLECT_POSITION = -19, Angle_COLLECT_POSITION = 13;
    public static double x_PLACE_MID = 35, y_PLACE_MID = -19, Angle_PLACE_MID = 13;
    public static double x_PARK1 = 11.5, y_PARK1 = -14, Angle_PARK1 = 0;
    public static double x_PARK2 = 33, y_PARK2 = -14.5, Angle_PARK2 = 0;
    public static double x_PARK3 = 60, y_PARK3 = -14, Angle_PARK3 = 0;
    public static double x_PreloadIntf = 36, y_PrelaodIntf = 0, Angle_PreloadIntf = -1;
    public static double x_Slide = 36 , y_Slide= -15;
    public static  double x_Slide2 = 36 , y_Slide2 = -5;
    public static double x_INTER = 35.5, y_INTER = -14 , Angle_PARK_INTER =0;
    double nrvolt =0;
    double okvolt =0, okprimucon;
    ElapsedTime asteapta = new ElapsedTime(), timerRetract = new ElapsedTime(), timerLift =new ElapsedTime() , timeCollect = new ElapsedTime(), timerSwitchLeft = new ElapsedTime(), timeSlide = new ElapsedTime(), tbegin = new ElapsedTime();
    ElapsedTime timer_sa_va_dau_la_muie = new ElapsedTime();
    ElapsedTime timer_sa_va_dau_la_muie2 = new ElapsedTime();
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
        timeOutBaby = 0.1;
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
        AutoController_IntfMID autoController_intfMID = new AutoController_IntfMID();
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



       autoController_intfMID.Cone_Stack_Level  =5;
       autoController_intfMID.AutoLiftStatus = LiftController.LiftStatus.HIGH;
        autoController_intfMID.LimitLift = 0.6;
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
        TrajectoryVelocityConstraint PATH_1_CONSTRAINT = getVelocityConstraint(50, 45, TRACK_WIDTH);
        ElapsedTime timeStart = new ElapsedTime() , timeTurnPlace = new ElapsedTime(), timerGOPLACE = new ElapsedTime(), timerSlide = new ElapsedTime(), TimerWait = new ElapsedTime();
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        Pose2d PRELOAD = new Pose2d(x_PreloadIntf,y_PrelaodIntf,Math.toRadians(Angle_PreloadIntf));
        Pose2d SLD1 = new Pose2d(x_Slide, y_Slide, Math.toRadians(270));
        Pose2d SLD2 = new Pose2d(x_Slide2, y_Slide2, Math.toRadians(270));
        Pose2d Place_Mid = new Pose2d(x_PLACE_MID, y_PLACE_MID, Math.toRadians(Angle_PLACE_MID));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.PULA_MEA;
        TrajectorySequence SLIDE1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(x_Slide,y_Slide))
                .build();

        TrajectorySequence SLIDE2 = drive.trajectorySequenceBuilder(SLD1)
                .lineTo(new Vector2d(x_Slide2,y_Slide2))
                .build();

        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(SLD2)
                .lineToLinearHeading(new Pose2d(x_PreloadIntf,y_PrelaodIntf,Math.toRadians(Angle_PreloadIntf)))
                .build();

        TrajectorySequence PLACE_MID = drive.trajectorySequenceBuilder(PRELOAD)
                .lineToLinearHeading(new Pose2d(x_PLACE_MID, y_PLACE_MID, Math.toRadians(Angle_PLACE_MID)))
                .build();

        TrajectorySequence PlacePreloadOnMid = drive.trajectorySequenceBuilder(PRELOAD)
                .lineToLinearHeading(Place_Mid)
                .build();

        TrajectorySequence PARK1 = drive.trajectorySequenceBuilder(Place_Mid)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK1,y_PARK1,Math.toRadians(Angle_PARK1)))
                .build();
        TrajectorySequence PARK2 = drive.trajectorySequenceBuilder(Place_Mid)
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))

                .lineToLinearHeading(new Pose2d(x_PARK2,y_PARK2,Math.toRadians(Angle_PARK2)))
                .build();
        TrajectorySequence PARK3 = drive.trajectorySequenceBuilder(Place_Mid)
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
        tbegin.reset();
        nrvolt =0;

        camera.closeCameraDevice();
        if (isStopRequested()) return;
        int OKEIUT = 0;
        while (opModeIsActive() && !isStopRequested())
        {
            int ColectarePosition = robot.motorColectareStanga.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista
            switch (status) {
                case PULA_MEA: {
                    drive.followTrajectorySequenceAsync(SLIDE1);
                    timer_sa_va_dau_la_muie.reset();
                    status = STROBOT.Slide1gen;
                    break;
                }
                case Slide1gen: {

                    if(timer_sa_va_dau_la_muie.seconds()>1)
                    {drive.followTrajectorySequenceAsync(SLIDE2);
                        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.SA_VA_DAU_LA_MUIE;
                        timer_sa_va_dau_la_muie2.reset();
                        status = STROBOT.Slide2gen;}
                    break;
                }

                case Slide2gen: {
                    if(timer_sa_va_dau_la_muie2.seconds()>0.5) {
                        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED_0;
                        status = STROBOT.START;
                    }
                    break;
                }

                case START: {
                    if (!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                        status = STROBOT.GET_LIFT_UP;
                    }
                    break;
                }
                case GET_LIFT_UP: {
                    if(tbegin.seconds() <28)
                    {OKEIUT=0;
                        if (nr==0)
                        {
                            if (!drive.isBusy()) {
                                liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                                timerLift.reset();
                                status = STROBOT.VOLTAGE_CHECK;
                            }
                        }
                        else {
                            if (autoController_intfMID.CurrentStatus == NOTHING &&autoController_intfMID.timerLIFT.seconds() > 0.1) {
                                liftController.CurrentStatus = LiftController.LiftStatus.MIDAUTO;
                                timerLift.reset();
                                status = STROBOT.GET_LIFT_DOWN;
                            }

                            else if(autoController_intfMID.CurrentStatus == NO_CONE)
                            {
                                motorColectareController.NrConAuto = motorColectareController.NrConAuto +1;
                                if(nr>1){nr--;autoController_intfMID.Cone_Stack_Level = autoController_intfMID.Cone_Stack_Level +1;}
                                else { nr =1;autoController_intfMID.Cone_Stack_Level = autoController_intfMID.Cone_Stack_Level +1;}
                                status = STROBOT.GO_TO_COLLECTING_POSITION;

                            }
                        }
                    }
                    else {
                        status = STROBOT.PARK;
                       autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.FORCED_PARK;
                    }
                    break;
                }

                case GET_LIFT_DOWN_FOR_PRELOAD_ON_SOUTH: {
                    if(timerGOPLACE.seconds()> 0.75)
                    {liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        status = STROBOT.GO_TO_COLLECTING_POSITION;}
                    break;
                }

                case VOLTAGE_CHECK: {

                    if(robot.dreaptaLift.getCurrent(CurrentUnit.AMPS) > 5 && liftController.CurrentPosition > 430)
                    {
                       autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.GET_LIFT_DOWN_WAIT;
                        TimerWait.reset();
                        status = STROBOT.GET_LIFT_UP_INTF_VOLTAGE;
                        nrvolt=nrvolt+1;
                    } else if(liftController.CurrentPosition > 600)
                    {status = STROBOT.GET_LIFT_DOWN_INTF;
                        okvolt=1;}
                    break;
                }
                case GET_LIFT_UP_INTF_VOLTAGE :{
                    if(TimerWait.seconds() > 0.1)
                    {
                        status = STROBOT.GO_TO_SCORING_POSITION_FOR_PRELOAD;
                        timerGOPLACE.reset();
                    }
                    break;
                }

                case GET_LIFT_DOWN: {
                    if (timerLift.seconds() >autoController_intfMID.LimitLift) {
                        timerLift.reset();
                        if (nr!=5)
                        {
                            liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                            turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                            status = STROBOT.GO_TO_COLLECTING_POSITION;
                        }
                        else
                        {
                            turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                            liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                            status = STROBOT.PARK;
                        }
                    }
                    break;
                }

                case GET_LIFT_DOWN_INTF: {
                    if (timerLift.seconds() >autoController_intfMID.LimitLift) {
                        timerLift.reset();

                        liftController.CurrentStatus = LiftController.LiftStatus.BASE_INTF;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                        status = STROBOT.GO_TO_COLLECTING_POSITION;}
                    break;
                }

                case COLLECT:
                { if( tbegin.seconds() < 28) {
                    if (nr == 0 && timeCollect.seconds() > 2 && okvolt == 1) {
                        nr++;
                       autoController_intfMID.Limit4Bar = 0.55;
                       autoController_intfMID.LimitSiguranta = 0.7;
                       autoController_intfMID.LimitOpenClaw = 0.85;
                       autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.STACK_LEVEL;
                        status = STROBOT.GO_TO_SCORING_POSITION;
                    } else if (nr > 0 && timeCollect.seconds() > 0.4) {
                        nr++;
                       autoController_intfMID.Limit4Bar = 0.55;
                       autoController_intfMID.LimitSiguranta = 0.7;
                       autoController_intfMID.LimitOpenClaw = 0.85;
                       autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.STACK_LEVEL;
                        status = STROBOT.GO_TO_SCORING_POSITION;
                    } else if(nr == 0 && timeCollect.seconds() > 0.6 && okvolt == 0)
                    {
                        nr++;
                       autoController_intfMID.Limit4Bar = 0.55;
                       autoController_intfMID.LimitSiguranta = 0.7;
                       autoController_intfMID.LimitOpenClaw = 0.85;
                       autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.STACK_LEVEL;
                        status = STROBOT.GO_TO_SCORING_POSITION;
                    }
                } else
                {
                    status = STROBOT.PARK;
                   autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.FORCED_PARK;
                }  break;
                }

                case GO_TO_SCORING_POSITION_FOR_PRELOAD:
                {
                    drive.followTrajectorySequenceAsync(PlacePreloadOnMid);
                    if(timerGOPLACE.seconds()> 0.1) {
                        status = STROBOT.GET_LIFT_UP_PRELOAD_ON_SOUTH;
                        timerGOPLACE.reset();
                    }
                    break;
                }

                case GET_LIFT_UP_PRELOAD_ON_SOUTH:
                {
                    if(timerGOPLACE.seconds() > 2.5)
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                        status = STROBOT.GET_LIFT_DOWN_FOR_PRELOAD_ON_SOUTH;
                        timerGOPLACE.reset();
                    }

                    break;
                }

                case GO_TO_SCORING_POSITION:
                { if(tbegin.seconds() < 28)
                {if (autoController_intfMID.CurrentStatus == PLACE_THE_CONE ||autoController_intfMID.CurrentStatus == NOTHING)
                {
                    if (nr<=5)
                    {

                        timerGOPLACE.reset();
                        status = STROBOT.GET_LIFT_UP;

                    }
                    else
                    {
                        if (nr == 11)
                        {
                            drive.followTrajectorySequenceAsync(PlacePreloadOnMid);
                        }
                        else
                        {
                            drive.followTrajectorySequenceAsync(PlacePreloadOnMid);
                        }
                        timerGOPLACE.reset();
                        status = STROBOT.GET_LIFT_UP;
                    }
                }
                } else {
                    status = STROBOT.PARK;
                   autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.FORCED_PARK;
                }
                    break;
                }




                case GO_TO_COLLECTING_POSITION:
                { if(tbegin.seconds() < 28)
                {motorColectareController.NrConAuto = motorColectareController.NrConAuto-1;
                    if (motorColectareController.NrConAuto == -1) {
                        motorColectareController.NrConAuto = 9;
                    }
                    if(nr==0)
                        drive.followTrajectorySequenceAsync(PLACE_MID);
                    timeCollect.reset();
                    status = STROBOT.COLLECT;} else {
                    status = STROBOT.PARK;
                   autoController_intfMID.CurrentStatus = AutoController_IntfMID.autocontroller_intfMID.FORCED_PARK;
                }
                    break;
                }
                case PARK:
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
           autoController_intfMID.update(sigurantaLiftController,robot,angle4BarController, turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);

            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            telemetry.addData("nr", nr);
            telemetry.addData("limit4bar",autoController_intfMID.Limit4Bar);
            // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.addData("curent", robot.motorColectareDreapta.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ColectareEncoder", motorColectareController.CurrentStatus);
            telemetry.addData("distanta", robot.dsensor.getDistance(DistanceUnit.MM));
            telemetry.addData("status", status);
            telemetry.addData("con",autoController_intfMID.Cone_Stack_Level);
            telemetry.update();;
        }
    }

}