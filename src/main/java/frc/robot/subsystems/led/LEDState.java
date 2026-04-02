package frc.robot.subsystems.led;
public enum LEDState {
    OFF(0),
    AUTO(1),      // rainbow
    TELEOP(2),    // gold shimmer / golden experience alternating
    ENDGAME(3),   // hollow purple
    SHOOT(4),     // fill bar red/green
    INTAKE(5);    // bright green blink
    public final int id;
    LEDState(int id) {
        this.id = id;
    }
}