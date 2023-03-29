package processor;

import grb.GurobiExecutor;
import parser.DocumentParser;
import parser.Document;
import parser.OutputDocument;
import shapes.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 26.03.23
 */
public class Processor {


    private GurobiExecutor executor;
    private final DocumentParser parser;
    private final static int M = 999999;
    private OutputDocument output;

    enum SortType {
        LeftToRight,
        RightToLeft,
        TopToBottom,
        BottomToTop
    }


    public Processor() {
        this.parser = new DocumentParser();
    }

    public OutputDocument processToOutputPre(String path) {

        parser.parseInputToDocument(path);
        Document input = parser.getParseDoc();
        String[] names = path.split("/");
        input.setName(names[names.length - 1]);
        return processToOutputPre(input.getName(), input.getMaster(), input.getSlaves(), input.getObstacles());
    }

    /**
     * Process all the information from the board.
     * Preparation process.
     *
     * @param caseName  the name of the case
     * @param master    master
     * @param slaves    ArrayList of slaves
     * @param obstacles ArrayList of obstacles
     * @return OutputDocument from preparation
     */
    public OutputDocument processToOutputPre(String caseName, PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) {
        OutputDocument output = new OutputDocument(caseName);
        /*
        Determine the pseudo Variables of Master and slaves
         */
        pseudoBaseVariablesDetermination(master, slaves, obstacles);
        /*
        Determine the overlapped Obstacle within the path area between arbitrary two corners of two obstacles.
         */
        detectOverlappedObstacleWithinPathArea(obstacles);


        for (Obstacle cur_o : obstacles){

            for (Obstacle other_o : obstacles){

                if (!other_o.getName().equals(cur_o.getName())){

                    //LowerLeft Case
                    Map<BaseType, Path> lowerLeftPathMap = new HashMap<>();
                    cur_o.addToMapLowerLeftPath(other_o, lowerLeftPathMap);
                    for (BaseType type : cur_o.getMapLowerLeftBypassOs().get(other_o).keySet()){
                        Path path = new Path();
                        path.addToNodes(cur_o.getLowerLeft());
                        if (type == BaseType.lowerLeft) {
                            if (cur_o.getMapLowerLeftBypassOs().get(other_o).get(type).size() == 0) {
                                path.addToNodes(other_o.getLowerLeft());
                            }






                        }


                        lowerLeftPathMap.put(type, path);
                    }




                }



            }

        }



        /*
        Debug
         */
        /*for (Obstacle o : obstacles) {
            System.out.println(o);
        }*/


        return output;
    }


    public void sortedObstacles(ArrayList<Obstacle> targetObstacles, SortType type) {

        if (type == SortType.LeftToRight) {
            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onLeft(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }

        } else if (type == SortType.RightToLeft) {

            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onRight(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }
        } else if (type == SortType.TopToBottom) {

            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onTop(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }
        } else if (type == SortType.BottomToTop) {

            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onBottom(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }
        }


    }

    /**
     * Determine the overlapped obstacles within the PathArea
     *
     * @param obstacles ArrayList of obstacles
     */
    public void detectOverlappedObstacleWithinPathArea(ArrayList<Obstacle> obstacles) {

        for (int i = 0; i < obstacles.size(); ++i) {
            Obstacle cur_o = obstacles.get(i);

            for (int j = 0; j < obstacles.size(); ++j) {
                Obstacle other_o = obstacles.get(j);
                if (!other_o.getName().equals(cur_o.getName())) {
                    Map<BaseType, ArrayList<Obstacle>> typeToArrayMap = new HashMap<>();
                    //cur_o's LowerLeft
                    typeToArrayMap.put(BaseType.lowerLeft, overlappedObstacles(obstacles, cur_o.getLowerLeft(), other_o.getLowerLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.lowerRight, overlappedObstacles(obstacles, cur_o.getLowerLeft(), other_o.getLowerRight(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperLeft, overlappedObstacles(obstacles, cur_o.getLowerLeft(), other_o.getUpperLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperRight, overlappedObstacles(obstacles, cur_o.getLowerLeft(), other_o.getUpperRight(), cur_o, other_o));
                    cur_o.addToMapLowerLeftBypassOs(other_o, typeToArrayMap);


                    //cur_o's LowerRight
                    typeToArrayMap = new HashMap<>();
                    typeToArrayMap.put(BaseType.lowerLeft, overlappedObstacles(obstacles, cur_o.getLowerRight(), other_o.getLowerLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.lowerRight, overlappedObstacles(obstacles, cur_o.getLowerRight(), other_o.getLowerRight(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperLeft, overlappedObstacles(obstacles, cur_o.getLowerRight(), other_o.getUpperLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperRight, overlappedObstacles(obstacles, cur_o.getLowerRight(), other_o.getUpperRight(), cur_o, other_o));
                    cur_o.addToMapLowerRightBypassOs(other_o, typeToArrayMap);

                    //cur_o's UpperLeft
                    typeToArrayMap = new HashMap<>();
                    typeToArrayMap.put(BaseType.lowerLeft, overlappedObstacles(obstacles, cur_o.getUpperLeft(), other_o.getLowerLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.lowerRight, overlappedObstacles(obstacles, cur_o.getUpperLeft(), other_o.getLowerRight(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperLeft, overlappedObstacles(obstacles, cur_o.getUpperLeft(), other_o.getUpperLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperRight, overlappedObstacles(obstacles, cur_o.getUpperLeft(), other_o.getUpperRight(), cur_o, other_o));
                    cur_o.addToMapUpperLeftBypassOs(other_o, typeToArrayMap);

                    //cur_o's UpperRight
                    typeToArrayMap = new HashMap<>();
                    typeToArrayMap.put(BaseType.lowerLeft, overlappedObstacles(obstacles, cur_o.getUpperRight(), other_o.getLowerLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.lowerRight, overlappedObstacles(obstacles, cur_o.getUpperRight(), other_o.getLowerRight(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperLeft, overlappedObstacles(obstacles, cur_o.getUpperRight(), other_o.getUpperLeft(), cur_o, other_o));
                    typeToArrayMap.put(BaseType.upperRight, overlappedObstacles(obstacles, cur_o.getUpperRight(), other_o.getUpperRight(), cur_o, other_o));
                    cur_o.addToMapUpperRightBypassOs(other_o, typeToArrayMap);


                }


            }

        }

    }

    private ArrayList<Obstacle> overlappedObstacles(ArrayList<Obstacle> obstacles, PseudoBase cur_node, PseudoBase other_node, Obstacle cur_o, Obstacle other_o) {
        PseudoBase p1 = cur_node;
        PseudoBase p3 = other_node;
        PseudoBase p2 = null, p4 = null;
        int dx = Math.abs(cur_node.getX() - other_node.getX());
        int dy = Math.abs(cur_node.getY() - other_node.getY());
        if (dx == dy || cur_node.getX() == other_node.getX() || cur_node.getY() == other_node.getY()) {
            p2 = cur_node;
            p4 = other_node;
        } else {
            int plusDY = (int) Math.max(dy * Math.signum(dx - dy), 0);
            int plusDx = (int) Math.max(dx * Math.signum(dy - dx), 0);
            //Case1: other -> lowerRight
            if (other_node.getX() > cur_node.getX() && other_node.getY() < cur_node.getY()) {
                p2 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() - plusDx);
                p4 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
            }
            //Case2: other -> upperRight
            else if (other_node.getX() > cur_node.getX() && other_node.getY() > cur_node.getY()) {
                p2 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
                p4 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() + plusDx);
            }
            //Case3: other -> upperLeft
            else if (other_node.getX() < cur_node.getX() && other_node.getY() > cur_node.getY()) {
                p4 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() - plusDx);
                p2 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() + plusDx);
            }
            //Case 4: other -> lowerLeft
            else if (other_node.getX() < cur_node.getX() && other_node.getY() < cur_node.getY()) {
                p4 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() - plusDx);
                p2 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() + plusDx);
            } else {
                System.err.println("Another Type of Parallelogram!!");
                System.out.println("cur_node = (" + cur_node.getX() + ", " + cur_node.getY() + "), other_node = (" + other_node.getX() + ", " + other_node.getY() + ")");
            }

        }

        ArrayList<Obstacle> overlappedO = new ArrayList<>();
        for (Obstacle o : obstacles) {
            //check if one of the edges of obstacles v.s., one of the edges of the parallelogram
            if (segmentOverlappedObstacle(p1, p2, overlappedO, o)) continue;
            if (segmentOverlappedObstacle(p2, p3, overlappedO, o)) continue;
            if (segmentOverlappedObstacle(p3, p4, overlappedO, o)) continue;
            if (segmentOverlappedObstacle(p4, p1, overlappedO, o)) continue;

        }
        //Left -- Right
        if (cur_node.getX() <= other_node.getX()) {
            if (cur_node.getType() == BaseType.lowerRight || cur_node.getType() == BaseType.upperRight) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.lowerLeft || other_node.getType() == BaseType.upperLeft) {
                overlappedO.remove(other_o);
            }
        } else {
            if (cur_node.getType() == BaseType.lowerLeft || cur_node.getType() == BaseType.upperLeft) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.lowerRight || other_node.getType() == BaseType.upperRight) {
                overlappedO.remove(other_o);
            }
        }
        //Up -- Down
        if (cur_node.getY() <= other_node.getY()) {
            if (cur_node.getType() == BaseType.upperLeft || cur_node.getType() == BaseType.upperRight) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.lowerLeft || other_node.getType() == BaseType.lowerRight) {
                overlappedO.remove(other_o);
            }
        } else {
            if (cur_node.getType() == BaseType.lowerLeft || cur_node.getType() == BaseType.lowerRight) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.upperLeft || other_node.getType() == BaseType.upperRight) {
                overlappedO.remove(other_o);
            }

        }

        return overlappedO;
    }

    private boolean segmentOverlappedObstacle(PseudoBase node1, PseudoBase node2, ArrayList<Obstacle> overlappedO, Obstacle o) {
        //p1_p2 vs o_LL_LR
        if (doIntersect(node1, node2, o.getLowerLeft(), o.getLowerRight())) {
            overlappedO.add(o);
            return true;
        }
        //p1_p2 vs o_LL_UL
        if (doIntersect(node1, node2, o.getLowerLeft(), o.getUpperLeft())) {
            overlappedO.add(o);
            return true;
        }
        //p1_p2 vs o_UR_UL
        if (doIntersect(node1, node2, o.getUpperRight(), o.getUpperLeft())) {
            overlappedO.add(o);
            return true;
        }
        //p1_p2 vs o_UR_LR
        if (doIntersect(node1, node2, o.getUpperRight(), o.getLowerRight())) {
            overlappedO.add(o);
            return true;
        }
        return false;
    }


    /**
     * Detect the direction and relation between known points, i.e., Master and Slaves, and obstacles.
     *
     * @param master    master
     * @param slaves    ArrayList of slaves
     * @param obstacles ArrayList of obstacles
     */
    public void pseudoBaseVariablesDetermination(PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) {

        for (Obstacle o : obstacles) {

            basicBinaryVariables(master, o);
            for (PseudoBase slave : slaves) {
                basicBinaryVariables(slave, o);
            }


        }


    }

    /**
     * Determine dir_q and rel_q
     *
     * @param base given point
     * @param o    given obstacle
     */
    private void basicBinaryVariables(PseudoBase base, Obstacle o) {

        /*
        Dir_q: L, R, A, B, UL, UR, LR, LL, D,U
         */
        int cnt_base_Dir_q = 10;
        int[] dir_q = new int[cnt_base_Dir_q];
        //L
        if (base.getX() < o.getMinX()) {
            dir_q[0] = 1;
        } else dir_q[0] = 0;
        //R
        if (base.getX() > o.getMaxX()) {
            dir_q[1] = 1;
        } else dir_q[1] = 0;
        //A
        if (base.getX() > o.getMaxY()) {
            dir_q[2] = 1;
        } else dir_q[2] = 0;
        //B
        if (base.getY() < o.getMinY()) {
            dir_q[3] = 1;
        } else dir_q[3] = 0;
        //UL
        if (base.getY() <= base.getX() + o.getMaxY() - o.getMinX()) {
            dir_q[4] = 1;
        } else dir_q[4] = 0;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()) {
            dir_q[5] = 1;
        } else dir_q[5] = 0;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()) {
            dir_q[6] = 1;
        } else dir_q[6] = 0;
        //LL
        if (base.getY() <= -base.getX() + o.getMinY() + o.getMinX()) {
            dir_q[7] = 1;
        } else dir_q[7] = 0;
        //D
        if (base.getY() <= (double) (o.getMinY() - o.getMaxY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMaxY()) {
            dir_q[8] = 1;
        } else dir_q[8] = 0;
        //U
        if (base.getY() <= (double) (o.getMaxY() - o.getMinY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMinY()) {
            dir_q[9] = 1;
        } else dir_q[9] = 0;
        base.addToPseudo_Dir_qs(o, dir_q);

        /*
        Rel_q:
         */
        int cnt_Rel_q = 8;
        int[] rel_q = new int[cnt_Rel_q];
        //Ld
        if (dir_q[0] == 1 && dir_q[2] + dir_q[3] == 0) {
            rel_q[0] = 1;
            base.addToOLd(o);
        } else rel_q[0] = 0;
        //Rd
        if (dir_q[1] == 1 && dir_q[2] + dir_q[3] == 0) {
            rel_q[1] = 1;
            base.addToORd(o);
        } else rel_q[1] = 0;
        //Ad
        if (dir_q[2] == 1 && dir_q[0] + dir_q[1] == 0) {
            rel_q[2] = 1;
            base.addToOAd(o);
        } else rel_q[2] = 0;
        //Bd
        if (dir_q[3] == 1 && dir_q[0] + dir_q[1] == 0) {
            rel_q[3] = 1;
            base.addToOBd(o);
        } else rel_q[3] = 0;
        //UpperLeft
        if (dir_q[7] + dir_q[9] == 0 && dir_q[5] == 1) {
            rel_q[4] = 1;
            base.addToOULd(o);
        } else rel_q[4] = 0;
        //UpperRight
        if (dir_q[6] + dir_q[8] == 0 && dir_q[4] == 1) {
            rel_q[5] = 1;
            base.addToOURd(o);
        } else rel_q[5] = 0;
        //LowerLeft
        if (dir_q[4] + dir_q[8] == 2 && dir_q[6] == 0) {
            rel_q[6] = 1;
            base.addToOLLd(o);
        } else rel_q[6] = 0;
        //LowerRight
        if (dir_q[5] + dir_q[9] == 2 && dir_q[7] == 0) {
            rel_q[7] = 1;
            base.addToOLRd(o);
        } else rel_q[7] = 0;
        base.addToPseudo_Rel_qs(o, rel_q);

    }


    public void pathDeterminationAlgorithm(ArrayList<Obstacle> obstacles) {


    }


    public static boolean onSegment(PseudoBase p, PseudoBase q, PseudoBase r) {
        if (q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX())
                && q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY()))
            return true;
        return false;
    }

    public static int orientation(PseudoBase p, PseudoBase q, PseudoBase r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) - (q.getX() - p.getX()) * (r.getY() - q.getY());

        if (val == 0)
            return 0;
        return (val > 0) ? 1 : 2;
    }

    // The main function that returns true if line segment 'p1q1' and 'p2q2' intersect.
    public static boolean doIntersect(PseudoBase p1, PseudoBase q1, PseudoBase p2, PseudoBase q2) {

        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        if (o1 != o2 && o3 != o4)
            return true;

        if (o1 == 0 && onSegment(p1, p2, q1))
            return true;

        if (o2 == 0 && onSegment(p1, q2, q1))
            return true;

        if (o3 == 0 && onSegment(p2, p1, q2))
            return true;

        if (o4 == 0 && onSegment(p2, q1, q2))
            return true;

        return false;
    }

}
