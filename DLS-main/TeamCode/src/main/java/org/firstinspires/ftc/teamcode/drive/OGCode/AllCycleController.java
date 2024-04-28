package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.CLOSE_CLAW;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.CLOSE_SAFETY_CLIP;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.EXTENDED_COLLECTION;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.GET_LIFT_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.RAISE_LIFT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.RELEASE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AllCycleController.AllCycleControllerStatus.RETRACT_COLLECTION;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.Place_Cone_Position;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AllCycleController {
    enum AllCycleControllerStatus
    {
        NOTHING,
        EXTENDED_COLLECTION,
        CLOSE_CLAW,
        RETRACT_COLLECTION,
        PLACE_CONE,
        CLOSE_SAFETY_CLIP,
        RELEASE_CONE,
        RAISE_LIFT,
        GET_LIFT_DOWN,
    }
    ElapsedTime extened_Collection_Timer = new ElapsedTime() , closeClaw_Timer = new ElapsedTime() , timeReleaseClaw = new ElapsedTime() , timeCloseSiguranta = new ElapsedTime();
    ElapsedTime timeReleaseCone = new ElapsedTime() ,timeLift = new ElapsedTime();
    AllCycleControllerStatus CurrentStatus = NOTHING;
    void update(RobotMap Robotel, SigurantaLiftController sigurantaLiftController, Angle4BarController angle4BarController, TurnClawController turnClawController, LiftController liftController, Servo4BarController servo4BarController, RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
    {
        switch (CurrentStatus)
        {
            case EXTENDED_COLLECTION:
            {
                sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
                motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.EXTENDED;
                robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
                extened_Collection_Timer.reset();
                CurrentStatus = CLOSE_CLAW;
                break;
            }
            case CLOSE_CLAW:
            {
                if (extened_Collection_Timer.seconds()>0.75)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    closeClaw_Timer.reset();
                    CurrentStatus =  RETRACT_COLLECTION;
                }
                break;
            }
            case RETRACT_COLLECTION:
            {
                if (closeClaw_Timer.seconds() > 0.45)
                {
                    motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE;

                    CurrentStatus = PLACE_CONE;
                }
                break;
            }
            case PLACE_CONE:
            {
                if (Math.abs(motorColectareController.CurrentPosition) < 15)
                {
                    Robotel.right4Bar.setPosition(Place_Cone_Position);
                    Robotel.left4Bar.setPosition(Place_Cone_Position);
                    timeReleaseClaw.reset();
                    CurrentStatus = CLOSE_SAFETY_CLIP;
                }
                break;
            }
            case CLOSE_SAFETY_CLIP:
            {
                if (timeReleaseClaw.seconds() > 0.75)
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    timeCloseSiguranta.reset();
                    CurrentStatus = RELEASE_CONE;
                }
                break;
            }
            case RELEASE_CONE:
            {
                if (timeCloseSiguranta.seconds() > 0.25)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                    angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.VERTICAL;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;

                    timeReleaseCone.reset();

                    CurrentStatus = RAISE_LIFT;
                }
                break;
            }
            case RAISE_LIFT:
            {
                if (timeReleaseCone.seconds()>0.1)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.HIGH;
                    timeLift.reset();
                    CurrentStatus = GET_LIFT_DOWN;
                }
                break;
            }
            case GET_LIFT_DOWN:
            {
                if (timeLift.seconds() > 0.85)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    CurrentStatus = EXTENDED_COLLECTION;
                }
                break;
            }
        }
    }
}
