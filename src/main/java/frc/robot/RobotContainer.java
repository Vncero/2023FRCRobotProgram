package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.indexer.Indexer;

public class RobotContainer {
    private final XboxController driverController = new XboxController(0);
    private final XboxController manipulatorController = new XboxController(1);

    private Indexer indexer = new Indexer();

    public RobotContainer() {
        // indexer.setDefaultCommand();
    }
}
