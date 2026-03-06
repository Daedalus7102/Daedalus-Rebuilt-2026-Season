package frc.robot.subsystems.shooter;

public class LookUpTable {

	public record DataPoint(double distance, double rpm, double angle) {
	}

	public static DataPoint[] table = {
			new DataPoint(0, 0, 0),
			new DataPoint(1, 1000, 1)
	};

	/**
	 * Gets an interpolated DataPoint from the table
	 * if the value is out of the table range, it will return the last value.
	 *
	 * @param distance the distance from the target
	 * @return a DataPoint with the RPM and hood angle
	 */
	public static DataPoint getPoint(double distance) {

		if (distance <= table[0].distance) {
			return table[0];
		} else if (distance >= table[table.length - 1].distance) {
			return table[table.length - 1];
		}

		for (int i = 0; i < table.length - 1; i++) {
			if (distance >= table[i].distance && distance <= table[i + 1].distance) {
				DataPoint a = table[i];
				DataPoint b = table[i + 1];
				return interpolatePoint(
						a,
						b,
						distance
				);
			}
		}
		return table[table.length - 1];
	}

	private static DataPoint interpolatePoint(DataPoint a, DataPoint b, double desiredDistance) {

		double t = (desiredDistance - a.distance) / (b.distance - a.distance);

		return new DataPoint(
				lerp(a.distance, b.distance, t),
				lerp(a.rpm, b.rpm, t),
				lerp(a.angle, b.angle, t)
		);
	}

	private static double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}
}