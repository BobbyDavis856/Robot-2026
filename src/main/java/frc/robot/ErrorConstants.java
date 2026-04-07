package frc.robot;

import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.logging.HealthSubsystem.ErrorCode;

public final class ErrorConstants {
    public static final ErrorCode JOYSTICKS_DISCONNECTED = new ErrorCode(
        50,
        "No joysticks connected",
        LEDPattern.steps(Map.of(0, Color.kPurple, 0.5, Color.kRed)),
        true,
        true
    );

    public static final ErrorCode DS_DISCONNECTED = new ErrorCode(
        100,
        "No driver station connection",
        LEDPattern.solid(new Color(255, 0, 0)).blink(Second.of(0.5)),
        true,
        true
    );

    public static final ErrorCode SPARKMAX_CAN_ERROR = new ErrorCode(
        25,
        "A sparkmax has a CAN error",
        LEDPattern.steps(Map.of(0, Color.kGreen, 0.5, Color.kYellow)),
        false,
        true
    );

    public static final ErrorCode LIMELIGHT_DISCONNECTED = new ErrorCode(
        15,
        "A limelight is disconnected",
        LEDPattern.steps(Map.of(0, Color.kGreen, 0.5, Color.kBlue)),
        false,
        true
    );
}
