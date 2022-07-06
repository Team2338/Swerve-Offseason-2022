package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
    private static Indexer instance = null;

    private static CANSparkMax indexerMotor;
    private static WPI_TalonSRX indexerStopperMotor = new WPI_TalonSRX(RobotMap.INDEXER_STOPPER_MOTOR);
    public static WPI_TalonSRX singulatorMotor = new WPI_TalonSRX(RobotMap.SINGULATOR_MOTOR);
    private static WPI_TalonSRX collectorMotor = new WPI_TalonSRX(RobotMap.COLLECTOR_MOTOR);

    private static final DigitalInput sensor1 = new DigitalInput(RobotMap.SENSOR_ONE);
    private static final DigitalInput sensor2 = new DigitalInput(RobotMap.SENSOR_TWO);

    public static Indexer getInstance() {
        if (instance == null) {
            System.out.println("indexer init");
            instance = new Indexer();
        }
        return instance;
    }

    /** Creates a new Indexer. */
    public Indexer() {
        super();

        indexerMotor = new CANSparkMax(RobotMap.INDEXER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        //indexerStopperMotor = new WPI_TalonSRX(RobotMap.INDEXER_STOPPER_MOTOR);

        indexerMotor.restoreFactoryDefaults();
        indexerMotor.setInverted(false);
        indexerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        indexerStopperMotor.configFactoryDefault();
        indexerStopperMotor.setInverted(true);
        indexerStopperMotor.setNeutralMode(NeutralMode.Brake);

        collectorMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setSpeedIndexer(double percent) {
        indexerMotor.set(percent);
    }

    public void setSpeedIndexerStopper(double percent) {
        indexerStopperMotor.set(percent);
    }

    public void setSpeedSingulator(double percent) {
        singulatorMotor.set(percent);
    }

    public void setSpeedCollector(double percent) {
        collectorMotor.set(percent);
    }

    public boolean getStateOne() {
        return sensor1.get();
    }

    public boolean getStateTwo() {
        return sensor2.get();
    }
}
