package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.RobotContainer;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.subsystems.ManipCRX;

public class CmdManager {
    private static NAR_XboxController controller = RobotContainer.controller;
    private static ManipCRX manipulator = ManipCRX.getInstance();

    private CmdManager() {}

    public static CommandBase CmdManipIntake(boolean cone) {
        return sequence(
            runOnce(() -> manipulator.intake(cone), manipulator),
            waitSeconds(0.4),
            waitUntil(() -> manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller(), manipulator)
        );
    }

    public static CommandBase CmdManipStop() {
        return runOnce(()-> manipulator.stopRoller(), manipulator);
    }

    public static CommandBase CmdManipOutake() {
        return sequence(
            runOnce(() -> manipulator.outtake(), manipulator),
            waitSeconds(0.4),
            waitUntil(() -> !manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller(), manipulator)
        );
    }

    public static CommandBase CmdManipShoot() {
        return sequence(
            runOnce(() -> manipulator.shoot(), manipulator),
            waitUntil(() -> !manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller(), manipulator)
        );
    }
    public static CommandBase vibrateController() {
        return new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())));
    }
}