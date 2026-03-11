package frc.robot.subsystems.shooter;

public class LookUpTable {

	public record DataPoint(double distance, double rpm, double angle) {
	}

	public static DataPoint[] table = {
            new DataPoint(2.38, 3640, 10),
			new DataPoint(2.61, 3750, 10),
			new DataPoint(2.79, 3900, 10.8),
			new DataPoint(2.97, 3950, 12.2),
			new DataPoint(3.22, 3950, 12.8),
			new DataPoint(3.38, 3950, 13.15),
			new DataPoint(3.61, 3970, 13.3),
			new DataPoint(3.83, 3980, 13.5),
			new DataPoint(4.1, 4100, 13.8),
			new DataPoint(4.2, 4080, 14),
			new DataPoint(4.41, 4200, 14),
			new DataPoint(4.61, 4330, 14.5),
			new DataPoint(4.825, 4420, 14.5),
			new DataPoint(5.03, 4500, 14.5),
			new DataPoint(5.21, 4580, 15),
			new DataPoint(5.51, 4630, 15),
			new DataPoint(5.81, 4700, 15)
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