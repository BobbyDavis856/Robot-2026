package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ClimbIOReal implements ClimbIO {
    private final SparkMax climbMotor;
    private final SparkMaxConfig climbConfig;

    private final DigitalInput climbedSensor;

    public ClimbIOReal() {
        climbMotor = new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);

        climbConfig = new SparkMaxConfig();
        climbConfig.idleMode(IdleMode.kBrake);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        climbedSensor = new DigitalInput(Constants.ClimbConstants.CLIMB_SENSOR_DIO);
    }

    @Override
    public void setClimbMotorVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    @Override
    public boolean getClimbedSensor() {
        return !climbedSensor.get();
    }
}
