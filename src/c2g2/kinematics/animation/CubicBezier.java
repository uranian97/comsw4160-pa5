package c2g2.kinematics.animation;

import org.joml.Matrix3d;
import org.joml.Vector2d;

import java.util.ArrayList;

/* Cubic Bezier interpolation of angles. 
 * You want to take into account of all frames and use two consecutive frames 
 * to specify the tangent direction of the control points. */
public class CubicBezier extends Interpolator {

	// Add more variables if needed.
	int numAngles;
	int numTimes;

	public CubicBezier(ArrayList<double[]> frames, ArrayList<Integer> times, int fps) {
		super(frames, times, fps);
		numAngles = super.kfAngles.get(0).length;
		numTimes = times.get(times.size()-1);
		int degree = times.size()-1;
	}

	@Override
	public double[] interp(double t) {
		// TODO: implement cubic bezier interpolation.

		double[] vals = spline(numTimes,t,super.kfAngles,0, numTimes);

		return vals;

	}

	Vector2d pointInLine(Vector2d A, Vector2d B, float T) {
		Vector2d C = new Vector2d();
		C.x = A.x - ((A.x - B.x) * T);
		C.y = A.y - ((A.y - B.y) * T);
		return C;
	}


	//https://rosettacode.org/wiki/Map_range
	private double toTimeInstant(double s){
		int a1 = 0;
		int a2 = 1;
		int b1 = 0;
		int b2 = numTimes;
		double t = b1 + ((s-a1)*(b2-b1))/(a2-a1);
		return t;
	}

	// https://pomax.github.io/bezierinfo/
	private double[] cubBezier(double t, ArrayList<double[]> weights){
		int n = 3;
		double t2 = t * t;
		double t3 = t2 * t;
		double mt = 1-t;
		double mt2 = mt * mt;
		double mt3 = mt2 * mt;
		double[] beziers = new double[weights.get(0).length];
		for(int i = 0; i<weights.get(0).length-1; i++){
			beziers[i] =  weights.get(0)[i] * mt3 +
					n*weights.get(1)[i] * mt2*t +
					n* weights.get(2)[i] * mt*t2 +
					weights.get(3)[i]*t3;
		}

		return beziers;
	}

	private double[] spline(int degree, double t, ArrayList<double[]> frames, int startFrame, int endFrame){
		if(degree < 4){
			ArrayList<double[]> weights;
			double[] cb = new double[numAngles];
			if(endFrame == numTimes) {
				weights = new ArrayList<>(frames.subList(startFrame, endFrame+1));
				/*double[] bez = cubBezier(t, weights);

				for(int i = 0; i<cb.length; i++){
					if(i == startFrame) cb[i] = bez[0];
					else if(i== startFrame + 1) cb[i] = bez[1];
					else if(i == endFrame) cb[i] = bez[3];
					else if(i == endFrame-1) cb[i] = bez[2];
					else cb[i] = 0;

				}*/
			}
			else{
				weights = new ArrayList<>();
				weights.add(frames.get(startFrame));
				weights.add(frames.get(startFrame+1));
				weights.add(frames.get(endFrame));
				weights.add(frames.get(endFrame-1));

				/*double[] bez = cubBezier(t, weights);

				for(int i = 0; i<cb.length; i++){
					if(i == startFrame) cb[i] = bez[0];
					else if(i== startFrame + 1) cb[i] = bez[1];
					else if(i == endFrame) cb[i] = bez[2];
					else if(i == endFrame-1) cb[i] = bez[3];
					else cb[i] = 0;
				}
				*/

			}

			//return cb;
			return cubBezier(t, weights);
		}

		double[] C = spline(degree - 1, t, frames, startFrame, startFrame + (degree-1) );
		double[] D = spline(degree - 1, t, frames, endFrame-(degree), endFrame);
		double[] P = C.clone();

		for(int i = 0; i < P.length; i++){
			P[i] = P[i] * (1-t) + (t * D[i]);
		}

		return P;
	}


	//t is time instant form
	private double getB(int i, double t){
		if(i == 1){
			if(i <= t && t < i+1) return 1;
			else return 0;
		}
		else{
			return (t/(i-1)) * getB(i-1,t) + ((i-t)/(i-1)) * getB(i-1, t-1);
		}
	}
}