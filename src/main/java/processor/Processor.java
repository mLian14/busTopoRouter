package processor;

import grb.GurobiExecutor;
import grb.GurobiVariable;
import gurobi.GRB;
import gurobi.GRBException;
import parser.DocumentParser;
import parser.Document;
import parser.OutputDocument;
import shapeVar.VirtualPointVar;
import shapes.*;

import java.util.*;

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

    enum OppositeType {
        topLtoBottomR,
        bottomLtoTopR,
        topRtoBottomL,
        bottomRtoTopL,
        NoOppositeRelation
    }


    public Processor() {
        this.parser = new DocumentParser();
    }

    public OutputDocument processToOutputPre(String path) throws GRBException {

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
    public OutputDocument processToOutputPre(String caseName, PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) throws GRBException {
        OutputDocument output = new OutputDocument(caseName);
        /*
        Determine the pseudo Variables of Master and slaves
         */
        pseudoBaseVariablesDetermination(master, slaves, obstacles);

        executor = new GurobiExecutor("LinearBusRouting_KO");
        executor.setMIPGap(0);
        executor.setMIPGapAbs(0);

        //build Gurobi Varibales








        /*
        Debug
         */
        /*for (Obstacle o : obstacles) {
            System.out.println(o);
        }*/


        return output;
    }

    public void buildVars(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualpointVars, ArrayList<PseudoBase> slaves, PseudoBase master) {
        GurobiVariable[] qs;

        /*
        find the boundary of the board
         */
        ArrayList<Integer> xs = new ArrayList<>();
        ArrayList<Integer> ys = new ArrayList<>();
        for (Obstacle o : obstacles) {
            xs.add(o.getMinX());
            xs.add(o.getMaxX());
            ys.add(o.getMinY());
            ys.add(o.getMaxY());
        }
        for (PseudoBase sv : slaves) {
            xs.add(sv.getX());
            ys.add(sv.getY());
        }
        xs.add(master.getX());
        ys.add(master.getY());

        int lb_x = Collections.min(xs);
        int ub_x = Collections.max(xs);
        int lb_y = Collections.min(ys);
        int ub_y = Collections.max(ys);
        //lb_x
        if (lb_x <= 0) {
            lb_x *= 2;
        } else {
            lb_x *= 0.5;
        }
        //lb_y
        if (lb_y <= 0) {
            lb_y *= 2;
        } else {
            lb_y *= 0.5;
        }
        //ub_x
        if (ub_x >= 0) {
            ub_x *= 2;
        } else {
            ub_x *= 0.5;
        }
        //ub_y
        if (ub_y >= 0) {
            ub_y *= 2;
        } else {
            ub_y *= 0.5;
        }

        int lb = Math.min(lb_x, lb_y);
        int ub = Math.max(ub_x, ub_y);



        /*
        initialize virtual points
         */
        for (int i = 0; i < slaves.size(); ++i) {
            VirtualPointVar vp = new VirtualPointVar();
            vp.x = new GurobiVariable(GRB.INTEGER, lb_x, ub_x, "x" + i);
            executor.addVariable(vp.x);
            vp.y = new GurobiVariable(GRB.INTEGER, lb_y, ub_y, "y" + i);
            executor.addVariable(vp.y);


            for (Obstacle o : obstacles) {
                /*
                o_vp_Non_qs
                0: nonL
                1: nonR
                2: nonA
                3: nonB
                 */
                vp.o_vp_Non_qs.put(o, buildObstacleBinaryVar(i, o, "o_vp_Non_qs", 4));

                /*
                o_vp_Rel_qs
                0: ul
                1: ur
                2: lr
                3: ll
                4: d
                5: u
                 */
                vp.o_vp_Rel_qs.put(o, buildObstacleBinaryVar(i, o, "o_vp_Rel_qs", 6));

                /*
                o_vp_diagonalSets_qs
                0: o_tL
                1: o_tR
                2: o_bL
                3: o_bR
                 */
                vp.o_vp_diagonalSets_qs.put(o, buildObstacleBinaryVar(i, o, "o_vp_diagonalSets_qs", 4));

                /*
                o_vp_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: relative obstacle: aux.2
                 */
                vp.o_vp_relObstacles_qs.put(o, buildObstacleBinaryVar(i, o, "o_vp_relObstacles_qs", 5));


                /*
                o_vvCorner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                vp.o_vvCorner_qs.put(o, buildObstacleBinaryVar(i, o, "o_vvCorner_qs", 4));

                /*
                vv_inOutCnn_qs
                0: ->
                1: <-
                */
                vp.vv_inOutCnn_qs.put(o, buildObstacleBinaryVar(i, o, "vv_inOutCnn_qs", 2));


                /*
                vv_oCoordinate_iqs
                0: x_m
                1: y_m
                 */
                vp.vv_oCoordinate_iqs.put(o, buildObstacleIntVar(i, o, lb, ub, "vv_oCoordinate_iqs", 2));



                /*
                vv_ooCnn_qs
                0: q_ij^m->n
                 */
                Map<Obstacle, GurobiVariable[]> obstacleToGrbArrayMap = new HashMap<>();
                for (Obstacle other_o : obstacles){

                    int cnt_vv_ooCnn_qs = 1;
                    qs = new GurobiVariable[cnt_vv_ooCnn_qs];
                    for (int var_cnt = 0; var_cnt < cnt_vv_ooCnn_qs; ++var_cnt) {
                        qs[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + o.getName() + "->" + other_o.getName() + "_" + "vv_ooCnn_qs" + "_" + var_cnt);
                        executor.addVariable(qs[var_cnt]);
                    }
                    obstacleToGrbArrayMap.put(other_o, qs);
                }
                vp.vv_ooCnn_qs.put(o, obstacleToGrbArrayMap);

            }

            /*
            vp_detour_qs
            0: ul->lr
            1: lr->ul
            2: ur->ll
            3: ll->ur
            4: detour trigger: aux.3
             */
            int cnt_vp_detour_qs = 5;
            vp.vvDetour_qs = buildBinaryVar(i, "", cnt_vp_detour_qs);


        }


    }

    private GurobiVariable[] buildObstacleIntVar(int i, Obstacle o, int lb, int ub, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.INTEGER, lb, ub, "v_" + i + ";" + o.getName() + "_" + varName + "_" + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }


    private GurobiVariable[] buildBinaryVar(int i, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";_" + varName + "_" + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }

    private GurobiVariable[] buildObstacleBinaryVar(int i, Obstacle o, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + o.getName() + "_" + varName + "_" + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }


    public OppositeType relationDetermineRegardOs(ArrayList<Obstacle> relevantObstacles, PseudoBase cur_n, PseudoBase other_n) {

        OppositeType type = OppositeType.NoOppositeRelation;
        for (Obstacle cur_o : relevantObstacles) {
            for (Obstacle other_o : relevantObstacles) {
                //topL to bottomR
                if (cur_o.topL_bottomR_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[4] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[7] == 1) {
                    type = OppositeType.topLtoBottomR;
                    break;
                }
                //bottomR to topL
                if (cur_o.bottomR_topL_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[7] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[4] == 1) {
                    type = OppositeType.bottomRtoTopL;
                    break;
                }
                //topR to bottomL
                if (cur_o.topR_bottomL_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[5] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[6] == 1) {
                    type = OppositeType.topRtoBottomL;
                    break;
                }
                //bottomL to topR
                if (cur_o.bottomL_topR_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[6] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[5] == 1) {
                    type = OppositeType.bottomLtoTopR;
                }


            }
        }


        return type;
    }

    /**
     * Sorted ArrayList of Obstacles according to the SortType
     *
     * @param targetObstacles ArrayList of Obstacles
     * @param type            SortType
     */
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

    private ArrayList<PseudoBase> parallelogramClockwise(PseudoBase cur_node, PseudoBase other_node) {
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
        ArrayList<PseudoBase> array = new ArrayList<PseudoBase>(Arrays.asList(p1, p2, p3, p4));
        return array;
    }

    private ArrayList<Obstacle> overlappedObstacles(ArrayList<Obstacle> obstacles, PseudoBase cur_node, PseudoBase other_node, PseudoBase bypassNode, Obstacle cur_o, Obstacle other_o) {

//        PseudoBase p1 = cur_node;
//        PseudoBase p3 = other_node;
//        PseudoBase p2 = null, p4 = null;
//        int dx = Math.abs(cur_node.getX() - other_node.getX());
//        int dy = Math.abs(cur_node.getY() - other_node.getY());
//        if (dx == dy || cur_node.getX() == other_node.getX() || cur_node.getY() == other_node.getY()) {
//            p2 = cur_node;
//            p4 = other_node;
//        } else {
//            int plusDY = (int) Math.max(dy * Math.signum(dx - dy), 0);
//            int plusDx = (int) Math.max(dx * Math.signum(dy - dx), 0);
//            //Case1: other -> lowerRight
//            if (other_node.getX() > cur_node.getX() && other_node.getY() < cur_node.getY()) {
//                p2 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() - plusDx);
//                p4 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
//            }
//            //Case2: other -> upperRight
//            else if (other_node.getX() > cur_node.getX() && other_node.getY() > cur_node.getY()) {
//                p2 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
//                p4 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() + plusDx);
//            }
//            //Case3: other -> upperLeft
//            else if (other_node.getX() < cur_node.getX() && other_node.getY() > cur_node.getY()) {
//                p4 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() - plusDx);
//                p2 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() + plusDx);
//            }
//            //Case 4: other -> lowerLeft
//            else if (other_node.getX() < cur_node.getX() && other_node.getY() < cur_node.getY()) {
//                p4 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() - plusDx);
//                p2 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() + plusDx);
//            } else {
//                System.err.println("Another Type of Parallelogram!!");
//                System.out.println("cur_node = (" + cur_node.getX() + ", " + cur_node.getY() + "), other_node = (" + other_node.getX() + ", " + other_node.getY() + ")");
//            }
//
//        }

        ArrayList<Obstacle> overlappedO = new ArrayList<>();

        for (Obstacle o : obstacles) {
            //check if one of the edges of obstacles v.s., one of the edges of the parallelogram
            if (segmentOverlappedObstacle(cur_node, bypassNode, o)) {
                overlappedO.add(o);
            }
            if (segmentOverlappedObstacle(bypassNode, other_node, o)) {
                overlappedO.add(o);
            }
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

    private boolean segmentOverlappedObstacle(PseudoBase node1, PseudoBase node2, Obstacle o) {
        //node1_node2 vs o_LL_LR
        if (doIntersect(node1, node2, o.getLowerLeft(), o.getLowerRight())) {
            return true;
        }
        //node1_node2 vs o_LL_UL
        if (doIntersect(node1, node2, o.getLowerLeft(), o.getUpperLeft())) {
            return true;
        }
        //node1_node2 vs o_UR_UL
        if (doIntersect(node1, node2, o.getUpperRight(), o.getUpperLeft())) {
            return true;
        }
        //node1_node2 vs o_UR_LR
        return doIntersect(node1, node2, o.getUpperRight(), o.getLowerRight());
    }


    /**
     * Detect the direction and relation between known points, i.e., Master and Slaves, and obstacles.
     *
     * @param master    master
     * @param slaves    ArrayList of slaves
     * @param obstacles ArrayList of obstacles
     */
    public void pseudoBaseVariablesDetermination(PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) {

        ArrayList<PseudoBase> bases = new ArrayList<>(slaves);
        bases.add(master);
        for (Obstacle o : obstacles) {
            basicBinaryVariables(master, o, slaves);
            for (PseudoBase slave : slaves) {
                basicBinaryVariables(slave, o, slaves);
            }

            /*
            oo_dir
             */
            for (Obstacle other_o : obstacles) {
                if (!other_o.getName().equals(o.getName())) {

                    //OtL:
                    if (o.down_AreaOverlap(other_o) && o.onTop(other_o)) {
                        o.addTotLObstacles(other_o);
                    }

                    //OtR:
                    if (o.up_AreaOverlap(other_o) && o.onTop(other_o)) {
                        o.addTotRObstacles(other_o);
                    }

                    //ObL:
                    if (o.up_AreaOverlap(other_o) && o.onBottom(other_o)) {
                        o.addTobLObstacles(other_o);
                    }

                    //ObR:
                    if (o.down_AreaOverlap(other_o) && o.onBottom(other_o)) {
                        o.addTobRObstacles(other_o);
                    }


                } else continue;
            }
        }


    }

    /**
     * Determine dir_q and rel_q
     *
     * @param base given point
     * @param o    given obstacle
     */
    private void basicBinaryVariables(PseudoBase base, Obstacle o, ArrayList<PseudoBase> bases) {

        /*
        oDir_q: L, R, A, B, UL, UR, LR, LL, D,U
         */
        int cnt_oDir_q = 6;
        int[] odir_q = new int[cnt_oDir_q];
        //UL
        if (base.getY() <= base.getX() + o.getMaxY() - o.getMinX()) {
            odir_q[0] = 1;
        } else odir_q[0] = 0;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()) {
            odir_q[1] = 1;
        } else odir_q[1] = 0;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()) {
            odir_q[2] = 1;
        } else odir_q[2] = 0;
        //LL
        if (base.getY() <= -base.getX() + o.getMinY() + o.getMinX()) {
            odir_q[3] = 1;
        } else odir_q[3] = 0;
        //D
        if (base.getY() <= (double) (o.getMinY() - o.getMaxY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMaxY()) {
            odir_q[4] = 1;
        } else odir_q[4] = 0;
        //U
        if (base.getY() <= (double) (o.getMaxY() - o.getMinY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMinY()) {
            odir_q[5] = 1;
        } else odir_q[5] = 0;
        base.addToPseudo_oDir_qs(o, odir_q);

        /*
        oRel_q:
         */
        int cnt_oRel_q = 4;
        int[] orel_q = new int[cnt_oRel_q];
        //UpperLeft
        if (odir_q[2] + odir_q[4] == 0 && odir_q[1] == 1) {
            orel_q[0] = 1;
            base.addToOULd(o);
        } else orel_q[0] = 0;
        //UpperRight
        if (odir_q[2] + odir_q[4] == 0 && odir_q[0] == 1) {
            orel_q[1] = 1;
            base.addToOURd(o);
        } else orel_q[1] = 0;
        //LowerLeft
        if (odir_q[0] + odir_q[4] == 2 && odir_q[2] == 0) {
            orel_q[2] = 1;
            base.addToOLLd(o);
        } else orel_q[2] = 0;
        //LowerRight
        if (odir_q[1] + odir_q[5] == 2 && odir_q[3] == 0) {
            orel_q[3] = 1;
            base.addToOLRd(o);
        } else orel_q[3] = 0;
        base.addToPseudo_oRel_qs(o, orel_q);


    }


    public static boolean onSegment(PseudoBase p, PseudoBase q, PseudoBase r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX())
                && q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
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
