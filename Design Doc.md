# Structure
The code follows the standard wpilib convention of commands and subsystems. Subsystems have been split into folders containing the subsystems and io files related to the part of the robot.

```
src/main/
├── deploy/
│   
└── java/frc/robot/
    ├── commands/
    │   ├── intake/
    │   │   ├── ActiveIntakeCommand.java
    │   │   ├── DeployIntakeCommand.java
    │   │   ├── RetractIntakeCommand.java
    │   │   └── ToggleIntakeDeployCommand.java
    │   ├── kicker/
    │   │   ├── ActivateKickerCommand.java
    │   │   └── ReverseKickerCommand.java
    │   ├── smart/
    │   │   └── SmartShootCommand.java
    │   ├── spindexer/
    │   │   ├── ActivateSpindexerCommand.java
    │   │   └── ReverseSpindexerCommand.java
    │   └── turret/
    │       ├── ActivateShooterCommand.java
    │       ├── HomeTurretCommand.java
    │       ├── ManualAimCommand.java
    │       ├── ManualStowTurretCommand.java
    │       ├── ResetTurretPitchCommand.java
    │       ├── ToggleFixedAimCommand.java
    │       ├── ToggleManualAimCommand.java
    │       └── TurretAutoAimCommand.java
    ├── libraries/
    │   ├── control/
    │   │   ├── ControllerIO.java
    │   │   ├── ControllerIOPS5.java
    │   │   └── ControllerIOXbox.java
    │   ├── FieldHelpers.java
    │   ├── LimelightHelpers.java
    │   ├── PoseHelpers.java
    │   ├── ProjectileSimulation.java
    │   ├── StateMachine.java
    │   └── SubsystemStateMachine.java
    ├── subsystems/
    │   ├── drive/
    │   │   ├── SwerveIO.java
    │   │   ├── SwerveIOReal.java
    │   │   └── SwerveSubsystem.java
    │   ├── intake/
    │   │   ├── IntakeDeploymentIO.java
    │   │   ├── IntakeDeploymentIOReal.java
    │   │   ├── IntakeDeploymentSubsystem.java
    │   │   ├── IntakeIO.java
    │   │   ├── IntakeIOReal.java
    │   │   └── IntakeSubsystem.java
    │   ├── lights/
    │   │   └── LightSubsystem.java
    │   ├── logging/
    │   │   ├── HealthSubsystem.java
    │   │   └── VisualizerSubsystem.java
    │   ├── spindexer/
    │   │   ├── SpindexerIO.java
    │   │   ├── SpindexerIOReal.java
    │   │   └── SpindexerSubsystem.java
    │   ├── turret/
    │   │   ├── CalculationSubsystem.java
    │   │   ├── KickerIO.java
    │   │   ├── KickerIOReal.java
    │   │   ├── KickerSubsystem.java
    │   │   ├── ShooterIO.java
    │   │   ├── ShooterIOKrakenReal.java
    │   │   ├── ShooterIOReal.java
    │   │   ├── ShooterSubsystem.java
    │   │   ├── TurretIO.java
    │   │   ├── TurretIOKrakenReal.java
    │   │   ├── TurretIOReal.java
    │   │   └── TurretSubsystem.java
    │   └── vision/
    │       ├── LimelightSubsystem.java
    │       └── QuestNavSubsystem.java
    ├── Constants.java
    ├── ErrorConstants.java
    ├── Main.java
    ├── Robot.java
    └── RobotContainer.java
```

# IO abstraction layer

Each subsystem that controls hardware with the exception of `LimelightSubsystem.java`, `QuestNavSubsystem.java`, and `LightSubsystem.java` uses an IO abstraction layer. This has several advantages for the code:
 - It makes the subsystem code cleaner
 - It allows subsystems to be disabled by passing in the `____IO.java` instead of an `____IOReal.java`
 - It allows for the hardware to be easily switched for example `ShooterIOReal`(NEOs) and `ShooterIOKrakenReal.java`(Krakens)
 - For our team specificly it allows testing code on incomplete hardware

Each IO abstraction layer consists of a base `____IO.java` class and a class with the real implementation `____IOReal.java`. For example the spindexer uses `SpindexerIO.java` and `SpindexerIOReal.java`

### SpindexerIO.java:
```java
package frc.robot.subsystems.spindexer;

public interface SpindexerIO {
    default void setMotorVoltage(double voltage) {}

    default double getMotorCurrent() {return 0;}

    default boolean checkCANError() {return false;}
}
```

### SpindexerIOReal.java:
```java
package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class SpindexerIOReal implements SpindexerIO {
    private final SparkMaxConfig spindexerConfig;
    private final SparkMax spindexerMotor;

    public SpindexerIOReal() {
        spindexerMotor = new SparkMax(Constants.SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

        spindexerConfig = new SparkMaxConfig();
        spindexerConfig.idleMode(IdleMode.kBrake);
        spindexerConfig.inverted(Constants.SpindexerConstants.SPINDEXER_MOTOR_INVERTED);
        spindexerConfig.smartCurrentLimit(40); 
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        spindexerMotor.setVoltage(voltage);
    }

    @Override
    public double getMotorCurrent() {
        return spindexerMotor.getOutputCurrent();
    }

    @Override
    public boolean checkCANError() {
        spindexerMotor.getBusVoltage();
        if (spindexerMotor.getFaults().can == true) {
            return true;
        }

        return false;
    }
}
```

The `SpindexerIO.java` class is an interface that implments default functions. Default getter functions return safe values that won't crash the subsystem or cause adverse effects. Setter functions accept all of the arguments of the normal setter then discard the values and do nothing. In `SpindexerIOReal.java` each of the default methods is implemented via a `@Override`

# Statemachines

Each of the subsystems that controls hardware with the exception of `SwerveSubsystem.java`, `LimelightSubsystem.java`, `QuestNavSubsystem.java`, and `LightSubsystem.java` extends a custom statemachine called `SubsystemStateMachine` (Code can be found in `/src/main/java/frc/robot/libraries/SubsystemStateMachine.java`) which itself extends `SubsystemBase` and provides a proxy for the custom `StateMachine` (Code can be found in `/src/main/java/frc/robot/libraries/StateMachine.java`). State machines have several advantages for the code:
 - It centralizes the state switching logic
 - It adds a prioity system so multiple systems requesting a state get automatically prioritized
 - It reduces conditions where the robot goes into weird states that are hard to debug

The state machine operates on a `desiredState` and a `currentState`. The `desiredState` is the state that the subsystem wants to be in and the `currentState` is the state that the subsystem is in. When something requests a desired state it passes in both the `desiredState` and a `priority`. At the start of each periodic tick the state machine sets the current `desiredState` to the requested `desiredState` with the highest priority. In this code I have structured the priorities as follows:
 - 0: Idle state
 - 1-10: Normal commands
 - 11-20: Manual overrides
 - 21-30: Safety overrides

While the architecture is pretty elegant and worked very well there are some important things to remember:
 1. It takes one tick to switch `desiredState`. Meaning if a state is requested on tick 0 it will only be set as the `desiredState` on tick 1
 2. In the state machines default configuration it will automatically reset its `desiredState` to the `defaultState` if no requests for a `desiredState` are made in a tick. This behaviour can be disabled by passing in `null` to the `defaultState` upon constructing the state machine
 3. Only the subsystem that extends the `SubsystemStateMachine` can transition states using the `transitionTo()`. If you find yourself in a state where this needs to be violated you should probabley use the base `StateMachine` class

### SpindexerSubsystem.java:
```java
package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ErrorConstants;
import frc.robot.RobotContainer;
import frc.robot.libraries.SubsystemStateMachine;
import frc.robot.subsystems.turret.CalculationSubsystem.Zone;

public class SpindexerSubsystem extends SubsystemStateMachine<frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState> {

    public enum SpindexerState {
        IDLE,
        STOWED,
        READY_REVERSE,
        READY,
    }

    private final SpindexerIO io;

    private double lastErrorTimestamp = Double.NEGATIVE_INFINITY;

    public SpindexerSubsystem(SpindexerIO io) {
        super(SpindexerState.IDLE, SpindexerState.IDLE);

        if (io == null) {
            throw new IllegalArgumentException("SpindexerIO cannot be null");
        }

        this.io = io;
    }

    public Current getMotorCurrent() {
        return Amp.of(io.getMotorCurrent());
    }

    public void checkCanHealth() {
        double timestamp = Timer.getFPGATimestamp();
        if (io.checkCANError()) {
            lastErrorTimestamp = timestamp;
        }

        if ((timestamp - lastErrorTimestamp) < Constants.HealthConstants.CAN_ERROR_PERSIST.in(Second)) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        }
    }

    @Override
    public void statePeriodicBefore() {
        if (RobotContainer.calculationSubsystem.getZone() == Zone.TRENCH) {
            requestDesiredState(SpindexerState.STOWED, 30);
        } else {
            requestDesiredState(SpindexerState.IDLE, 0);
        }
    }

    @Override
    public void statePeriodic() {

        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == SpindexerState.STOWED) {
                    transitionTo(SpindexerState.STOWED);
                } else if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                }
                break;
            case STOWED:
                if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                } else if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }
                break;

            case READY_REVERSE:
                if (getDesiredState() == SpindexerState.STOWED) {
                    transitionTo(SpindexerState.STOWED);
                } else if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }

                break;
            case READY:
                if (getDesiredState() == SpindexerState.STOWED) {
                    transitionTo(SpindexerState.STOWED);
                } else if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }

                break;
        }

        double spindexerVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                spindexerVoltage = 0;
                break;
            case STOWED:
                spindexerVoltage = 0;
                break;
            case READY_REVERSE:
                spindexerVoltage = -Constants.SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE.in(Volt);
                break;
            case READY:
                spindexerVoltage = Constants.SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE.in(Volt);
                break;
            default:
                spindexerVoltage = 0.0;
                System.err.println("Spindexer in unknown state: " + getCurrentState());
                break;
        }

        spindexerVoltage = MathUtil.clamp(spindexerVoltage, -10, 10);
        io.setMotorVoltage(spindexerVoltage);

        checkCanHealth();

        SmartDashboard.putNumber("Spindexer/Voltage", spindexerVoltage);

        SmartDashboard.putNumber("Spindexer/Current", io.getMotorCurrent());

        SmartDashboard.putString("Spindexer/Current State", getCurrentState().name());
        SmartDashboard.putString("Spindexer/Desired State", getDesiredState().name());
    }
}
```

The class extends `SubsystemStateMachine` and passes a type of an `Enum` containing all the posible states of the subsystem. In the constructor of `SpindexerSubsystem.java` it calls the super of the `SubsystemStateMachine` and passes in the `startingState` and the `defaultState`. The `startingState` controls what state the state machine starts in and the `defaultState` controls what state the `desiredState` resets to if no `desiredState` is requested in a tick. The code calls `statePeriodicBefore` then the `statePeriodic`. `statePeriodicBefore` is called before the previous ticks `desiredState` is applied and `statePeriodic` is called after. In `statePeriodic` the subsystem evalulates if it should transition from each state to each other state. Then it executes the code for each current state.

# Units
WPILib provides a helpful `Units` library that allows unit safe varibles. This provides several advantages for the code:
 - Prevent mistakes involving incorect units in calculations
 - Automatically converts bettween units
 - Makes the codes intent much more explicit
 - Prevents passing in the wrong units into a function

The `Units` library provides a set of predefined unit types which includes `Distance`, `Angle`, `LinearVelocity`, `Time`, and `Mass`. These units are further split into specific units. For example to set a distance you would do:
```java
Distance exampleDistance = Meters.of(1.423);
```
Then to read that distance back into a double you would do:
```java
double exampleDistanceMeter = exampleDistance.in(Meters);
double exampleDistanceInches = exampleDistance.in(Inch);
```
I would strongly recomend you use the `Units` library in your code. You can find documentation [here](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html).

# Constants
This project uses a central file called `Constants.java` to store the constants for every part of the code. Just like how the subsystem files are organized into folders, the constants are organized into sub classes.
```java
 
