package org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_mid.autoControllerMidStatus.CLOSE_THE_CLAW;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_mid.autoControllerMidStatus.DRAW_MOTOR;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_mid.autoControllerMidStatus.FOURBAR_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_mid.autoControllerMidStatus.GATA;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_mid.autoControllerMidStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_mid.autoControllerMidStatus.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1_mid.autoControllerMidStatus.RETRIEVE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.DRAW_MOTOR_MID_1;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.MID_CU_INTF_IN_PULA_MEA;
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

public class AutoController5_1_mid {
    public enum autoControllerMidStatus
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
        GATA,
    }
    public static autoControllerMidStatus CurrentStatus = NOTHING, PreviousStatus = NOTHING;
    ElapsedTime timerFourBar = new ElapsedTime() ,timerStart = new ElapsedTime(),timerClaw = new ElapsedTime(), TimeTransfer = new ElapsedTime(), timerPlace_Cone = new ElapsedTime(), timerLift =new ElapsedTime(), timerSigurantaIntf =new ElapsedTime();
    public static int Cone_Stack_Level=5;
    public static double LimitLift = 1.5, Limit4Bar = 0.55, LimitSiguranta=0.7, LimitOpenClaw =0.85, pozitiacurenta;
    public static double POS[] = {0,435,455,495,520,555}, POS2[] = {0,420,405,435,455,455}, AMPS[]={0,0.7,0.7,0.7,1.6,2.4};
    public static int Nrc = 5;
    public static LiftController.LiftStatus AutoLiftStatus = LiftController.LiftStatus.HIGH_SOUTH;
    boolean moreThanOneStack = false;
    int ok=0,ok2=0,oki=0;
    double timerInter = 2,timeStart=0;
    public void update(SigurantaLiftController sigurantaLiftController, RobotMap Robotel, Angle4BarController angle4BarController, TurnClawController turnClawController, LiftController liftController, Servo4BarController servo4BarController, RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
    { double distance =  Robotel.dsensor.getDistance(DistanceUnit.MM);
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
            case FOURBAR_DOWN:
            {
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                timerFourBar.reset();
                CurrentStatus = DRAW_MOTOR;
                break;
            }

            case DRAW_MOTOR: {
                if(timerFourBar.seconds() > 0.3)
                {
                    motorColectareController.CurrentStatus = DRAW_MOTOR_MID_1;
                if((distance <= 30 && motorColectareController.CurrentPosition > 110 ) || timerFourBar.seconds() > 1.5) {
                    CurrentStatus = CLOSE_THE_CLAW;
                    pozitiacurenta = motorColectareController.CurrentPosition;
                }}
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
                if (timerClaw.seconds() >0.5) //timer_claw 0.5
                {
                    motorColectareController.CurrentStatus = RETRACTED_0;
                    timerPlace_Cone.reset();
                    ok = 0;
                    ok2 =0;
                    oki = 0;
                    CurrentStatus = PLACE_CONE;
                    timerFourBar.reset();
                }
                break;
            }
            case PLACE_CONE:
            {
                if (ok2 == 0 && MotorColectareController.CurrentPosition <=  pozitiacurenta - 10)
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE_AUTO;
                    ok2 = 1;
                }
                if (ok == 0 && MotorColectareController.CurrentPosition <= 70)
                {
                    Robotel.left4Bar.setPosition(servo4BarController.Place_Cone_Position);
                    Robotel.right4Bar.setPosition(servo4BarController.Place_Cone_Position);
                    ok = 1;
                }
                if (MotorColectareController.CurrentPosition <= 20)
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    TimeTransfer.reset();
                    oki=1;
                }
                if (timerFourBar.seconds()>0.9 && oki==1)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_SMALL;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    motorColectareController.CurrentStatus = RETRACTED_0;
                    CurrentStatus = GATA;
                }
                break;
            }

            case GATA:
            {
                CurrentStatus = NOTHING;
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
                motorColectareController.CurrentStatus = MID_CU_INTF_IN_PULA_MEA;
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_BIG;
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                if (Cone_Stack_Level==5)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position_South;
                    Cone_Stack_Level =4;
                }
                else if (Cone_Stack_Level==4)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fourth_Cone_Position_South;
                    Cone_Stack_Level =3;
                }
                else if (Cone_Stack_Level==3)
                {
                    servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position_South;
                    Cone_Stack_Level =2;
                }
                else if (Cone_Stack_Level==2)
                {
                    servo4BarController.Collect_Position = servo4BarController.Second_Cone_Position_South;
                    Cone_Stack_Level =1;
                }
                else if (Cone_Stack_Level==1)
                {
                    servo4BarController.Collect_Position = servo4BarController.Ground_Position_South;
                    Cone_Stack_Level =5;

                }

                timerStart.reset();
                CurrentStatus = FOURBAR_DOWN;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
