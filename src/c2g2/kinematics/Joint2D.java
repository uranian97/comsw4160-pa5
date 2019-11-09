package c2g2.kinematics;

import java.util.ArrayList;
import org.joml.Matrix3d;
import org.joml.Vector2d;
import org.joml.Vector3d;

/* 
 * Class for Joint. 
 * Feel free to change things to make your own implementation easier. 
 */
public class Joint2D {
    private double length = -1.0; 
    private double rotateAngle = 0.; 	// in radians
    private Vector2d position;
    private double angleChange = 0;
        
    private ArrayList<Joint2D> cJoints; 	// children joints
    private Joint2D pJoint = null; 		// parent joint
    private Matrix3d transform = null; 

    // default constructor
    public Joint2D() {
        cJoints = new ArrayList<Joint2D>();
    }

    // constructor for root joint
    public Joint2D(Vector2d pos) {
        this();
        position = pos;
        pJoint = null;
        transform = new Matrix3d();
    }

    // constructor for child joint
    public Joint2D(Joint2D p, double l, double a) {
        this();
        pJoint = p;
        length = l;
        rotateAngle = a;
        calcPos();
    }

    public void calcPos() {
        // construct transform matrix at this joint
        double cos = Math.cos(rotateAngle);
        double sin = Math.sin(rotateAngle);
        Vector2d p = pJoint.getPos();
        transform = new Matrix3d(cos, sin, 0,
                                -sin, cos, 0,
                                p.x, p.y, 1);
        Vector3d res = (new Vector3d(length, 0, 1)).mul(transform);
        position = new Vector2d(res.x, res.y);
    }


    public boolean isLeaf() { return cJoints.isEmpty(); } // is end effector
    public boolean isRoot() { return pJoint == null; }

    public Vector2d getPos() { return position; }

    public Vector2d getLocalPos() {
        if(isRoot()) return position;
        else return new Vector2d(length, 0);
    }

    public void setPos(double x, double y) { position = new Vector2d(x, y); }
    public void setPos(Vector2d newPos) { position = newPos; } 

    public void setLength(double len) { length = len; }
    public double getLength() { return length; }

    public void setRotateAngle(double a) { rotateAngle = a; }
    public double getRotateAngle() { return rotateAngle; }

    public void setRotationFromRelative(double angle){
        if(pJoint.isRoot()) setRotateAngle(angle);
        else setRotateAngle(pJoint.getRotateAngle() + angle);
    }

    public Joint2D getParentJoint() { return pJoint; }
    public void setParentJoint(Joint2D p) { pJoint = p; }

    public ArrayList<Joint2D> getChildJoints() { return cJoints; }
    public void addChildJoint(Joint2D c) { cJoints.add(c); }
    public void setAngleChange(double angle){
        angleChange = angle;
    }
    public double getAngleChange(){
        return angleChange;
    }
}
