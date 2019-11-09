package c2g2.kinematics;

import org.joml.Matrix3d;
import org.joml.Vector2d;
import org.joml.Vector3d;
import org.lwjgl.system.CallbackI;

import java.util.ArrayList;
import java.util.HashMap;


/*
 * Class that implements the forward Kinematics algorithm.
 */
public class ForwardKinematics {
    
    private Skeleton2D skeleton;
    // TODO: Feel free to add more variables as needed.
    private HashMap<Joint2D, Integer> angles;
    
    public ForwardKinematics(Skeleton2D ske) {
        if (ske == null) throw new NullPointerException("The provided skeleton is NULL!");
        skeleton = ske;

        // TODO:
        // Here you can create a map that maps parameter array (see updateState(double[]) below)
        // to individual rotational angles of all the joints on the skeleton
        Joint2D root = ske.getRoot();
        angles = new HashMap<>();
        assignParams();
    }

    public void updateState(double[] params) {
        // TODO: Implement the forward kinematics algorithm here

        System.out.println("updateState");
        Joint2D root = skeleton.getRoot();
        Matrix3d I = new Matrix3d(1.0,0.0,0.0,
                                  0.0,1.0,0.0,
                                  0.0,0.0,1.0);

        System.out.println(angles.get(root));
            update3(root, params);

    }

    private void assignParams(){
        ArrayList<Joint2D> joints = skeleton.getJoints();
        for(int i = 0; i<joints.size(); i++){
            angles.put(joints.get(i),i);

        }
    }

    private void update3(Joint2D joint, double[] params){


        if(!joint.isRoot()){

            joint.setRotateAngle(joint.getParentJoint().getRotateAngle() + params[angles.get(joint)]);
            //joint.setRotateAngle(joint.getRotateAngle() + params[angles.get(joint)]);
            joint.calcPos();
        }
        ArrayList<Joint2D> children = joint.getChildJoints();
        for(int i = 0; i<children.size(); i++){
            Joint2D child = children.get(i);
            //child.setRotateAngle(joint.getRotateAngle());
            update3(child, params);
        }
    }
}
