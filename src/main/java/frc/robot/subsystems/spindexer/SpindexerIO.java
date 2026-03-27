package frc.robot.subsystems.spindexer;

public interface SpindexerIO {
    default void setMotorVoltage(double voltage) {}

    default double getMotorCurrent() {return 0;}
}
