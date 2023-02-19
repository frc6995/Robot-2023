package frc.robot.driver;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.POIManager;
import frc.robot.subsystems.ArmS.ArmPosition;

public class CommandOperatorKeypad {
    private GenericHID m_hid;
    private Consumer<Pose2d> setDriveSetpoint;
    private Consumer<ArmPosition> setArmSetpoint;
    private NetworkTableEntry selectionEntry = NetworkTableInstance.getDefault().getEntry("/DriverDisplay/selection");
    public enum Button {
        kLeftGrid(1),
        kCenterGrid(2),
        kRightGrid(3),
        kHighLeft(4),
        kHighCenter(5),
        kHighRight(6),
        kMidLeft(7),
        kMidCenter(8),
        kMidRight(9),
        kLowLeft(10),
        kLowCenter(11),
        kLowRight(12),
        kKillSwitch(13),
        kEnterKey(14);
    
        public final int value;
    
        Button(int value) {
          this.value = value;
        }

      }
    public CommandOperatorKeypad(int port, Consumer<Pose2d> setDriveSetpoint, Consumer<ArmPosition> setArmSetpoint) {
        this.setDriveSetpoint = setDriveSetpoint;
        this.setArmSetpoint = setArmSetpoint;
        m_hid = new GenericHID(port);
        setupTrigger(leftGrid(), Button.kLowLeft, 8, 0);
        setupTrigger(leftGrid(), Button.kLowCenter, 7, 0);
        setupTrigger(leftGrid(), Button.kLowRight, 6, 0);
        setupTrigger(leftGrid(), Button.kMidLeft, 8, 1);
        setupTrigger(leftGrid(), Button.kMidCenter, 7, 1);
        setupTrigger(leftGrid(), Button.kMidRight, 6, 1);
        setupTrigger(leftGrid(), Button.kHighLeft, 8, 2);
        setupTrigger(leftGrid(), Button.kHighCenter, 7, 2);
        setupTrigger(leftGrid(), Button.kHighRight, 6, 2);

        setupTrigger(centerGrid(), Button.kLowLeft, 5, 0);
        setupTrigger(centerGrid(), Button.kLowCenter, 4, 0);
        setupTrigger(centerGrid(), Button.kLowRight, 3, 0);
        setupTrigger(centerGrid(), Button.kMidLeft, 5, 1);
        setupTrigger(centerGrid(), Button.kMidCenter, 4, 1);
        setupTrigger(centerGrid(), Button.kMidRight, 3, 1);
        setupTrigger(centerGrid(), Button.kHighLeft, 5, 2);
        setupTrigger(centerGrid(), Button.kHighCenter, 4, 2);
        setupTrigger(centerGrid(), Button.kHighRight, 3, 2);

        setupTrigger(rightGrid(), Button.kLowLeft, 2, 0);
        setupTrigger(rightGrid(), Button.kLowCenter, 1, 0);
        setupTrigger(rightGrid(), Button.kLowRight, 0, 0);
        setupTrigger(rightGrid(), Button.kMidLeft, 2, 1);
        setupTrigger(rightGrid(), Button.kMidCenter, 1, 1);
        setupTrigger(rightGrid(), Button.kMidRight, 0, 1);
        setupTrigger(rightGrid(), Button.kHighLeft, 2, 2);
        setupTrigger(rightGrid(), Button.kHighCenter, 1, 2);
        setupTrigger(rightGrid(), Button.kHighRight, 0, 2);


    }

    private void setupTrigger(Trigger grid, Button position, int columnInGrid, int row) {
        grid.and(key(position)).onTrue(
            setpointCommand(()->POIManager.ownCommunity().get(columnInGrid), ()->Constants.ArmConstants.STOW_POSITION,
                ((row) * 9) + columnInGrid
            ));
    }
    private Trigger key(Button key) {
        return m_hid.button(key.value, CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }
    public Trigger leftGrid() {
        return key(Button.kLeftGrid);
    }

    public Trigger centerGrid() {
        return key(Button.kCenterGrid);
    }

    public Trigger rightGrid() {
        return key(Button.kRightGrid);
    }

    private Command setpointCommand(Supplier<Pose2d> driveSetpoint, Supplier<ArmPosition> armSetpoint, int selectionNumber) {
        return new InstantCommand(()->{
            setDriveSetpoint.accept(driveSetpoint.get());
            setArmSetpoint.accept(armSetpoint.get());
            selectionEntry.setInteger(selectionNumber);
        }).ignoringDisable(true);
    }
}
