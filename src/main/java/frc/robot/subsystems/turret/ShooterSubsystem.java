package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.libraries.SubsystemStateMachine;

public class ShooterSubsystem extends SubsystemStateMachine<frc.robot.subsystems.turret.ShooterSubsystem.ShooterState> {
    private final double SHOOTER_THRESHOLD = 1; // RPS

    public enum ShooterState {
        IDLE,
        SPOOLING,
        READY
    }

    private final ShooterIO io;

    private double shooterTargetVelocity = 0;

    public ShooterSubsystem(ShooterIO io) {
        super(ShooterState.IDLE, null);

        this.io = io;
    }

    public void setTargetSpeed(AngularVelocity speed) {
        shooterTargetVelocity = speed.in(RotationsPerSecond);
    }

    public AngularVelocity getTargetSpeed() {
        return RotationsPerSecond.of(shooterTargetVelocity);
    }

    public AngularVelocity getSpeed() {
        return RotationsPerSecond.of((io.getMotor1RPS() + io.getMotor2RPS()) / 2.0);
    }

    public void resetShooter() {
        shooterTargetVelocity = 0;
    }

    @Override
    public void periodic() {
        updateDesiredState();

        // Safety Check as the desired state should only ever IDLE or READY
        if (getDesiredState() == ShooterState.SPOOLING) {
            requestDesiredState(ShooterState.IDLE, 4);
        }
        
        
        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == ShooterState.READY) {
                    transitionTo(ShooterState.SPOOLING);
                }
                break;
            case SPOOLING:
                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) < SHOOTER_THRESHOLD) {
                    transitionTo(ShooterState.READY);
                }

                break;
            case READY:
                if (getDesiredState() == ShooterState.IDLE) {
                    transitionTo(ShooterState.IDLE);
                } else if (Math.abs(getSpeed().in(RotationsPerSecond) - getTargetSpeed().in(RotationsPerSecond)) > (SHOOTER_THRESHOLD + 0.1)) {
                    transitionTo(ShooterState.SPOOLING);
                }

                break;
        }


        switch (getCurrentState()) {
            case IDLE:
                io.setMotorsVoltage(0.0);
                break;
            case SPOOLING:
                io.setClosedVelocity(shooterTargetVelocity);
                break;
            case READY:
                io.setClosedVelocity(shooterTargetVelocity);
                break;
        }

        //SmartDashboard.putNumber("Shooter/Motor Voltage", shooterVoltage);

        SmartDashboard.putNumber("Shooter/Motor 1 Speed", io.getMotor1RPS() * 60.0);
        SmartDashboard.putNumber("Shooter/Motor 2 Speed", io.getMotor2RPS() * 60.0);

        SmartDashboard.putNumber("Shooter/Target Speed", getTargetSpeed().in(RotationsPerSecond) * 60);

        SmartDashboard.putString("Shooter/Current State", getCurrentState().name());
        SmartDashboard.putString("Shooter/Desired State", getDesiredState().name());
    }
}
