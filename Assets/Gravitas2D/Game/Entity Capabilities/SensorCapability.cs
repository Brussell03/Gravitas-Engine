using Unity.Mathematics;

namespace GravitasDemo
{
	public enum SensorType { Optical = 0, Infrared = 1, Radar = 2 }

	public class SensorCapability : IEntityCapability
	{
		public SensorType Type { get; set; }
		public double Range { get; set; }
		public float FieldOfViewDegrees { get; set; } = 360f; // Default omnidirectional
		public float OrientationOffset { get; set; } = 0f;    // Offset relative to entity orientation
		public float Sensitivity { get; set; } = 1.0f;        // Tuning multiplier for detection strength
	}
}