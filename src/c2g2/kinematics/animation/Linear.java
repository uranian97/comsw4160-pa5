package c2g2.kinematics.animation;

import java.util.ArrayList;

/* Linear interpolation of angles. */
public class Linear extends Interpolator {

	private int duration; // the length of the animation
	private int N; // the number of total frames in the animation

	public Linear(ArrayList<double[]> frames, ArrayList<Integer> times, int fps) {
		super(frames, times, fps);
		duration = kfTimestamps[kfTimestamps.length-1] - kfTimestamps[0];
		N = duration * fps;
	}

	@Override
	public double[] interp(double t) {
		int whichFrame = (int) (t * N); 
		for (int i = 0; i < kfTimestamps.length; i++) {
			if ((kfTimestamps[i]-kfTimestamps[0]) * fps > whichFrame) {
				whichFrame = i;
				break;
			}
		}

		double[] prevAngles = kfAngles.get(whichFrame-1);
		double[] nextAngles = kfAngles.get(whichFrame);
		double[] itpResult = new double[prevAngles.length];

		t = (t * duration - kfTimestamps[whichFrame-1]+kfTimestamps[0]) /
			(kfTimestamps[whichFrame] - kfTimestamps[whichFrame-1]);

		for (int i = 0; i < itpResult.length; i++) {
			double a = prevAngles[i];
			double b = nextAngles[i];
			itpResult[i] = a + (b - a) * t;
		}

		return itpResult;
	}
}