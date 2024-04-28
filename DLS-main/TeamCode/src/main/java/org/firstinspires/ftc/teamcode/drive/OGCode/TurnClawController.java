package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController.TurnClawStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController.TurnClawStatus.PLACE;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurnClawController {
    public enum TurnClawStatus
    {
        INIT,
        COLLECT,
        PLACE,
        COLLECT_SOUTH,
    }
    public static TurnClawStatus CurrentStatus = INIT , PreviousStatus = INIT;
    public static double pozTurnClaw_COLLECT=0.691, pozTurnClaw_PLACE = 0.0135,pozTurnCollectSouthCycling = 0.65;
    public void update(RobotMap Robotel)
    {
        if (PreviousStatus != CurrentStatus)
        {
            switch (CurrentStatus)
            {
                case PLACE:
                {
                    Robotel.turnClaw.setPosition(pozTurnClaw_PLACE);
                    break;
                }
                case COLLECT:
                {
                    Robotel.turnClaw.setPosition(pozTurnClaw_COLLECT);
                    break;
                }
                case COLLECT_SOUTH:
                {
                    Robotel.turnClaw.setPosition(pozTurnCollectSouthCycling);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
