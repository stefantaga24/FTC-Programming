package org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.CLOSE_THE_CLAW;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.DRAW_MOTOR;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.EXTENDO_STUCK;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.EXTEND_MUISTILOR;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.FARA_CON;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.FOURBAR_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.RETRACT_IN_PULA_MEA;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.RETRIEVE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.STACK_LEVEL;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.STUCK_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.STUCK_CONE_VERIF;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoSouthHighJunction5_1.autoControllerSouthHigh.VERIFICARE_ANTI_MUISTI;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.DRAW_MOTOR1;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.DRAW_MOTOR2;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_DRIVE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_SOUTH;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_SOUTH_SIGUR;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED_0;

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

public class AutoSouthHighJunction5_1<PARCARE_FORTATA> {
    public enum autoControllerSouthHigh
    {
        NOTHING,
        RETRIEVE_CONE,
        STACK_LEVEL,
        FOURBAR_DOWN,
        LIFT_CONE,
        INITIALIZE,
        CLOSE_THE_CLAW,
        PLACE_CONE,
        RETRACTED,
        GET_LIFT_DOWN,
        GET_LIFT_DOWN_INTF,
        GET_LIFT_DOWN_WAIT,
        DRAW_MOTOR,
        PARCARE_FORTATA,
        STUCK_CONE,
        STUCK_CONE_VERIF,
        EXTENDO_STUCK,
        RETRACT_IN_PULA_MEA,
        EXTEND_MUISTILOR,
        VERIFICARE_ANTI_MUISTI,
        FARA_CON,
    }
    public static autoControllerSouthHigh CurrentStatus = NOTHING;
    public static autoControllerSouthHigh PreviousStatus = NOTHING;
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
            case PARCARE_FORTATA:
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
                CurrentStatus = DRAW_MOTOR;
                break;
            }

            case VERIFICARE_ANTI_MUISTI: {
                if(timerStart.seconds() > 0.5 && MotorColectareController.CurrentPosition < 240)
                { timer_stuck.reset();
                    CurrentStatus = RETRACT_IN_PULA_MEA;}
                else if (MotorColectareController.CurrentPosition > 240)
                    CurrentStatus = FOURBAR_DOWN;
                break;
            }

            case RETRACT_IN_PULA_MEA: {
                MotorColectareController.CurrentStatus = RETRACTED_0;
                if(timer_stuck.seconds() > 1)
                    CurrentStatus = EXTEND_MUISTILOR;
                break;
            }

            case EXTEND_MUISTILOR: {
                    MotorColectareController.CurrentStatus = EXTENDED_SOUTH_SIGUR;
                    timerStart.reset();
                    CurrentStatus = VERIFICARE_ANTI_MUISTI;

                    break;
            }

            case DRAW_MOTOR: {
                motorColectareController.CurrentStatus = DRAW_MOTOR1;
                if((distance < DIS[Cone_Stack_Level] &&  motorColectareController.CurrentPosition > 400) || timerFourBar.seconds() > 1) {
                    CurrentStatus = CLOSE_THE_CLAW;
                    pozitiacurenta = motorColectareController.CurrentPosition;
                }
                break;
            }

            case CLOSE_THE_CLAW:
            {

                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    Nrc = Nrc - 1;
                    timerClaw.reset();
                    timerSigurantaIntf.reset();
                    CurrentStatus = RETRIEVE_CONE;

                break;
            }
            case RETRIEVE_CONE:
            {
                if (ok == 0 && timerClaw.seconds() >0.2) // timer_claw 0.2
                {
                    ok=1;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.RAISED;
                }
                if (timerClaw.seconds() >0.3) //timer_claw 0.5
                {
                    motorColectareController.CurrentStatus = RETRACTED_0;
                    timerPlace_Cone.reset();
                    ok = 0;
                    ok2 =0;
                    CurrentStatus = PLACE_CONE;
                }
                break;
            }
            case PLACE_CONE:
            {
                if (ok2 == 0 && MotorColectareController.CurrentPosition <=  POS[Cone_Stack_Level] - 10)
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE_AUTO;
                    ok2 = 1;
                }
                if (ok == 0 && MotorColectareController.CurrentPosition <= 70)
                {
                    if(distance < 60)
                    { Robotel.left4Bar.setPosition(servo4BarController.Place_Cone_Position);
                    Robotel.right4Bar.setPosition(servo4BarController.Place_Cone_Position);
                    TimeTransfer.reset();
                    ok = 1; }
                    else {
                        CurrentStatus = FARA_CON;
                        timer_fara_con.reset();
                    }
                }
                if (MotorColectareController.CurrentPosition <= 20)
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                }
                if (MotorColectareController.CurrentPosition <= (pozitiacurenta - pozitiacurenta + 10) && TimeTransfer.seconds() >= 0.2)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_SMALL;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    timerLIFT.reset();
                    CurrentStatus = NOTHING;
                }
                break;
            }

            case FARA_CON: {
                if(timer_fara_con.seconds() > 0.5)
                {Cone_Stack_Level = Cone_Stack_Level +1;
                CurrentStatus = STACK_LEVEL;}
                break;
            }


            case GET_LIFT_DOWN:
            {
                if (timerLift.seconds()>LimitLift)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE_BAZAVAN;
                    CurrentStatus= NOTHING;
                }
                break;
            }

            case GET_LIFT_DOWN_WAIT:
            {

                    liftController.CurrentStatus = LiftController.LiftStatus.BASE_CU_SIGURANTA;
               CurrentStatus = NOTHING;
                       break;
            }

            case STUCK_CONE: {
                motorColectareController.CurrentStatus = EXTENDED_SOUTH_SIGUR;
        // Cone_Stack_Level = Cone_Stack_Level +1;
            }

            case GET_LIFT_DOWN_INTF:
            {
                if (timerLift.seconds()>LimitLift)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE_INTF;
                    CurrentStatus= NOTHING;
                }
                break;
            }
            case STACK_LEVEL:
            {
                ok=0;
                motorColectareController.CurrentStatus = EXTENDED_SOUTH_SIGUR;
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
                   // Cone_Stack_Level =3;
                }
                else if (Cone_Stack_Level==3)
                {
                    servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position_South;
                    //Cone_Stack_Level =2;
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
                CurrentStatus = VERIFICARE_ANTI_MUISTI;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
    }
}