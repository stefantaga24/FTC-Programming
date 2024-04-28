package org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers;


import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.CLOSE_THE_CLAW;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.EXTEND;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.FORCED_PARK;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.NO_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.PLACE_THE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.RETRACT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.RETRIVE_THE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.VERIF_DISTANCE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.VERIF_DISTANCE;
//import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.DRAW_MOTOR;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.NO_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.STACK_LEVEL;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.ROBOT_INTERFERENCE_VERIF;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.EXTEND_MUISTILOR;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController_IntfMID.autocontroller_intfMID.FOURBAR_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.RETRACT_IN_PULA_MEA;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.VERIFICARE_ANTI_MUISTI;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.DRAW_MOTOR1;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.DRAW_MOTOR_MID_1;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_COMMONHIGHINTERFERENCE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_SOUTH_SIGUR;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.MID_CU_INTF_IN_PULA_MEA;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED_0;

import android.telecom.CallAudioState;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.CloseClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.SigurantaLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController;

import java.util.concurrent.CompletionException;

import fi.iki.elonen.NanoHTTPD;

public class AutoController_IntfMID {
    public enum autocontroller_intfMID
    {
        NOTHING,
        INITIALIZE,
        FORCED_PARK,
        FOURBAR_DOWN,
        VERIF_DISTANCE,
        CLOSE_THE_CLAW,
        RETRIVE_THE_CONE,
        PLACE_THE_CONE,
        NO_CONE,
        STACK_LEVEL,
        ROBOT_INTERFERENCE_VERIF,
        RETRACT,
        EXTEND,
        GET_LIFT_DOWN_WAIT,
        GET_LIFT_DOWN_INTF,

    }
    public static autocontroller_intfMID CurrentStatus = NOTHING,PreviousStatus =NOTHING ;
    public ElapsedTime timerFourBar = new ElapsedTime() ,timerStart = new ElapsedTime(),timerClaw = new ElapsedTime(), TimeTransfer = new ElapsedTime(), timerPlace_Cone = new ElapsedTime(), timerLift =new ElapsedTime(), timerSigurantaIntf =new ElapsedTime(), timer_stuck = new ElapsedTime(), timerLIFT = new ElapsedTime(), timer_fara_con = new ElapsedTime();
    public static int Cone_Stack_Level=5;
    public static double LimitLift = 1.25, Limit4Bar = 0.55, LimitSiguranta=0.7, LimitOpenClaw =0.85, pozitiacurenta;
    public static double POS[] = {0,435,455,495,520,555}, POS2[] = {0,420,405,435,455,455}, AMPS[]={0,3.5,4,3.5,3.9,4}, DIS[]={0,42,42,42,42,42};
    public static int Nrc = 5;
    public static LiftController.LiftStatus AutoLiftStatus = LiftController.LiftStatus.HIGH_SOUTH;
    boolean moreThanOneStack = false;
    int ok=0,ok2=0;
    double timerInter = 2,timeStart=0;
    public void update(SigurantaLiftController sigurantaLiftController, RobotMap Robotel, Angle4BarController angle4BarController, TurnClawController turnClawController, LiftController liftController, Servo4BarController servo4BarController, RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
    {double distance =  Robotel.dsensor.getDistance(DistanceUnit.MM);
        switch (CurrentStatus)
        {
            case INITIALIZE:
            {
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
                motorColectareController.CurrentStatus = RETRACTED;
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
                liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                CurrentStatus = NOTHING;
                break;
            }

            case FORCED_PARK:
            {
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
                motorColectareController.CurrentStatus = RETRACTED;
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
                liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                CurrentStatus = NOTHING;
                break;
            }



             case FOURBAR_DOWN:
        {
            servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
            turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
            timerFourBar.reset();
            CurrentStatus = VERIF_DISTANCE;
            break;
        }

            case VERIF_DISTANCE: {
                MotorColectareController.CurrentStatus = DRAW_MOTOR_MID_1;
                if((distance <= 30 && MotorColectareController.CurrentPosition > 250) || timerFourBar.seconds()>1)
                {
                    CurrentStatus = CLOSE_THE_CLAW;
                }
                break;
            }

            case CLOSE_THE_CLAW: {
                CloseClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                timerClaw.reset();
                pozitiacurenta = MotorColectareController.CurrentPosition;
                CurrentStatus = RETRIVE_THE_CONE;

            }

            case RETRIVE_THE_CONE: {
                if(ok == 0 && timerClaw.seconds() > 0.2)
                {
                    ok=1;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.RAISED;
                }
                if(timerClaw.seconds() > 0.3)
                {motorColectareController.CurrentStatus = RETRACTED_0;
                    ok = 0;
                    ok2 =0;
                    CurrentStatus = PLACE_THE_CONE;
                }
            }

            case PLACE_THE_CONE: {
                if (ok2== 0 && MotorColectareController.CurrentPosition <= pozitiacurenta - 10)
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE_AUTO;
                    ok2 = 1;
                }
                if ( ok ==0 && MotorColectareController.CurrentPosition <= 70)
                {
                    if(distance < 60)
                    { Robotel.left4Bar.setPosition(servo4BarController.Place_Cone_Position);
                        Robotel.right4Bar.setPosition(servo4BarController.Place_Cone_Position);
                        TimeTransfer.reset();
                        ok = 1; }
                    else {
                        CurrentStatus = NO_CONE;
                        timer_fara_con.reset();
                    }
                }
                if (MotorColectareController.CurrentPosition <= 20)
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                }
                if (MotorColectareController.CurrentPosition <= 15 && TimeTransfer.seconds() >= 0.2)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_SMALL;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    timerLIFT.reset();
                    CurrentStatus = NOTHING;
                }
                break;
            }

            case GET_LIFT_DOWN_INTF:
            {
                if (timerLift.seconds()>LimitLift)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE_INTF;
                    CurrentStatus= AutoController_IntfMID.autocontroller_intfMID.NOTHING;
                }
                break;
            }

            case NO_CONE: {
                if(timer_fara_con.seconds() > 0.5)
                {Cone_Stack_Level = Cone_Stack_Level +1;
                   CurrentStatus = STACK_LEVEL;}
                break;
            }

            case GET_LIFT_DOWN_WAIT:
            {

                liftController.CurrentStatus = LiftController.LiftStatus.BASE_CU_SIGURANTA;
                CurrentStatus = autocontroller_intfMID.NOTHING;
                break;
            }

            case STACK_LEVEL:
            {
                ok=0;
                motorColectareController.CurrentStatus = MID_CU_INTF_IN_PULA_MEA;
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_BIG;
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                if (Cone_Stack_Level==5)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position_South;
                    //Cone_Stack_Level =4;
                }
                else if (Cone_Stack_Level==4)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fourth_Cone_Position_South;
                   //Cone_Stack_Level =3;
                }
                else if (Cone_Stack_Level==3)
                {
                    servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position_South;
                   // Cone_Stack_Level =2;
                }
                else if (Cone_Stack_Level==2)
                {
                    servo4BarController.Collect_Position = servo4BarController.Second_Cone_Position_South;
                    //Cone_Stack_Level =1;
                }
                else if (Cone_Stack_Level==1)
                {
                    servo4BarController.Collect_Position = servo4BarController.Ground_Position_South;
                    //Cone_Stack_Level =5;

                }
                Cone_Stack_Level--;
                if(Cone_Stack_Level == 0)
                    Cone_Stack_Level = 5;
                timerStart.reset();
                CurrentStatus = FOURBAR_DOWN;
                break;
            }

        }
        PreviousStatus = CurrentStatus;
    }
}