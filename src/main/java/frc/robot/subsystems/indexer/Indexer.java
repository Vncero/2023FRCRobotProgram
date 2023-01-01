package frc.robot.subsystems.indexer;

import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Queue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CIEColor;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.NFunction;

/*  TODOs:
 *  1. research CIE color space (DONE)
 *  2. implement color matching/general detection (DONE)
 *  3. represent state of indexer internally
 *  4. figure out what to do with sensor info
*/

public class Indexer extends SubsystemBase {

    // hardware
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);

    private DigitalInput topLimitSwitch = new DigitalInput(0);
    private DigitalInput bottomLimitSwitch = new DigitalInput(1);

    private CANSparkMax indexerMotor = new CANSparkMax(Constants.Indexer.kIndexerPort, MotorType.kBrushless);

    // color matching
    private Color determinedAllianceColor;

    private CIEColor rawDetection;
    private CIEColor prevRawDetection;
    private ColorMatchResult matchedColor;

    private ColorMatch matcher = new ColorMatch();

    // triggers
    private Trigger sensorIsConnected = new Trigger(() -> {
        return this.colorSensor.isConnected();
    });

    private Trigger sensorInProximity = new Trigger(() -> {
        return this.colorSensor.getProximity() >= Constants.Indexer.kProximityLimit;
    });

    // state
    private Queue<Color> indexedQueue = new PriorityQueue<Color>(3);

    public Indexer() {
        sensorIsConnected.and(sensorInProximity).whenActive(new RunCommand(this::index, this)
            .withInterrupt(this::indexFinished)
            .andThen(() -> this.indexerMotor.set(0), this)
        );
        matcher.addColorMatch(Constants.Indexer.kRedTarget);
        matcher.addColorMatch(Constants.Indexer.kBlueTarget);
        matcher.setConfidenceThreshold(Constants.Indexer.kConfidenceThreshold);
    }

    @Override
    public void periodic() {
        // log out colors & state?
        this.logData();
    }

    private void logData() {
        SmartDashboard.putNumber("Proximity", this.colorSensor.getProximity()); 
        SmartDashboard.updateValues();
    }

    private void index() {
        // update existing before considering new detections
        if (!this.topLimitSwitch.get()) { // no ball at top i.e. shooter fired
            this.indexedQueue.remove();
            this.indexerMotor.set(Constants.Indexer.kIndexerSpeed); // move balls up
        }

        // check color sensor for any new ball detections
        this.updateDetections();

        // new ball detected
        if (Objects.nonNull(this.matchedColor) && !(this.rawDetection.equals(this.prevRawDetection))) {
            // figures out whether to eject ball, moves ball up if there's space
            if (this.matchedColor.color.equals(this.indexedQueue.peek()) || this.matchedColor.color.equals(this.determinedAllianceColor)) {
                if (this.indexedQueue.size() < 2) this.indexerMotor.set(Constants.Indexer.kIndexerSpeed);
                this.indexedQueue.add(this.matchedColor.color);
            } else {
                this.indexerMotor.set(-Constants.Indexer.kIndexerSpeed);
            }
        }
    }

    private boolean indexFinished() {
        // either all balls have been moved or #updateDetections() could not find a color
        return (this.topLimitSwitch.get() || Objects.isNull(this.matchedColor));
    }

    private void updateDetections() {
        if (this.colorSensor.isConnected()) {
            this.prevRawDetection = this.rawDetection; // save previous detection
            this.rawDetection = this.colorSensor.getCIEColor();
            if (this.rawDetection.equals(this.prevRawDetection)) {
                this.matchedColor = null;
                return; // exit early if there is no new detection
            }
            this.matchedColor = matcher.matchColor(this.convertCIEXYZ2sRGB(this.rawDetection));
        } else {
            this.rawDetection = null;
            this.matchedColor = null;

            SmartDashboard.putBoolean("Color Sensor Is Connected:", this.colorSensor.isConnected());
        }
    }

    private Color convertCIEXYZ2sRGB(CIEColor cieColor) {           
        NFunction<Double, Double> calculateConversions = (coeffs) -> {
            assert Objects.nonNull(coeffs) && coeffs.length >= 3;
            return (coeffs[0] * cieColor.getX() + coeffs[1] * cieColor.getY() + coeffs[2] * cieColor.getZ());
        };

        return new Color(
            calculateConversions.apply(Constants.CIEXYZ2sRGBConversion.rx, Constants.CIEXYZ2sRGBConversion.ry, Constants.CIEXYZ2sRGBConversion.rz), 
            calculateConversions.apply(Constants.CIEXYZ2sRGBConversion.gx, Constants.CIEXYZ2sRGBConversion.gy, Constants.CIEXYZ2sRGBConversion.gz),
            calculateConversions.apply(Constants.CIEXYZ2sRGBConversion.bx, Constants.CIEXYZ2sRGBConversion.by, Constants.CIEXYZ2sRGBConversion.bz) 
        );
    }
}