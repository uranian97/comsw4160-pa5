package c2g2.kinematics.engine;

import c2g2.kinematics.*;
import java.util.ArrayList;
import java.util.Arrays;
import org.joml.Vector2d;
import org.lwjgl.system.CallbackI;

public class Scene {

    public Skeleton2D skeleton;

    private int xmlFileCount = 0;
    public int rotatingJointIndex;
    private Joint2D draggingJoint;



    public Scene() {
        rotatingJointIndex = 0;
        draggingJoint = null;
    }

    public void loadfromXML(String filename) {
        skeleton = new Skeleton2D(filename);
        ///
    }
    
    // Return a list of joint positions.
    public ArrayList<Vector2d> getJointPos() {
        ArrayList<Vector2d> posList = new ArrayList<Vector2d>();
        visitNodePos(skeleton.getRoot(), posList);
        return posList;
    }
    
    // Add parent-child pairs as needed by the renderer. 
    private void visitNodePos(Joint2D currentJoint, ArrayList<Vector2d> posList) {
        ArrayList<Joint2D> cJoints = currentJoint.getChildJoints();
        for (Joint2D cJoint: cJoints) {
            posList.add(currentJoint.getPos());
            posList.add(cJoint.getPos());
            if (cJoint.isLeaf()) { continue; }
            visitNodePos(cJoint, posList);
        }
    }

    // When you press 'P', this method is called. 
    public void printAnglesToScreen() {
        System.out.println(Arrays.toString(skeleton.getAngles()));
    }

    // When you press 'S', this method is called.
    public void savePoseToXML() {
        try {
            ++xmlFileCount;
            skeleton.printPoseToXML(xmlFileCount);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public double[] rotateUp(Joint2D joint){
        ArrayList<Joint2D> joints = skeleton.getJoints();
        int index = joints.indexOf(joint);

        //joint.setAngleChange(0.01);
        double[] params = skeleton.getAngles();
        params[index] = params[index] + 0.01;
        return params;
    }

    public double[] rotateDown(Joint2D joint){
        ArrayList<Joint2D> joints = skeleton.getJoints();
        int index = joints.indexOf(joint);

        //joint.setAngleChange(0.01);
        double[] params = skeleton.getAngles();
        params[index] = params[index] - 0.01;
        return params;
    }

    public boolean isDragging(){
        if(draggingJoint != null) return true;
        else return false;
    }

    public void startDraggin(Joint2D joint){
        draggingJoint = joint;
    }

    public void stopDragging(){
        draggingJoint = null;
    }

    public boolean isEffector(Joint2D joint){
        if(joint.isLeaf() && joint.getChildJoints().size() == 0) return true;
        else return false;
    }

    public Joint2D getJointAt(Vector2d pos){
        ArrayList<Joint2D> joints = skeleton.getJoints();
        Joint2D jointAt = null;
        for (Joint2D joint: joints) {
            if(joint.getPos().distance(pos)<0.05) jointAt = joint;
        }
        return jointAt;
    }
    public Joint2D getDraggingJoint(){
        return draggingJoint;
    }
    
}
