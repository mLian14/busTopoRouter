package processor;

import grb.*;
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
    private final static int M = 99999;
    private OutputDocument output;

    enum SortType {
        LeftToRight,
        RightToLeft,
        TopToBottom,
        BottomToTop
    }

    enum OppositeType {
        ul_lr,
        ll_ur,
        ur_ll,
        lr_ul,
        l_r,
        r_l,
        t_b,
        b_t,
        NoDetour
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
        int minDist = 1;
        /*
        Determine the pseudo Variables of Master and slaves
         */
        pseudoBaseVariablesDetermination(master, slaves, obstacles);


        /*
        update map_oo_dist
         */
        for (Obstacle o : obstacles) {
            o.getLowerLeft().addToPseudo_oRel_qs(o, new int[]{0, 0, 1, 0, 1, 0, 0, 1});
            o.getLowerRight().addToPseudo_oRel_qs(o, new int[]{0, 0, 0, 1, 0, 1, 0, 1});
            o.getUpperLeft().addToPseudo_oRel_qs(o, new int[]{1, 0, 0, 0, 1, 0, 1, 0});
            o.getUpperRight().addToPseudo_oRel_qs(o, new int[]{0, 1, 0, 0, 0, 1, 1, 0});
            for (PseudoBase corner : o.getBaseArray()) {
                for (Obstacle on : obstacles) {
                    if (!on.getName().equals(o.getName())) {
                        basicBinaryVariables(corner, on);
                    }
                }
            }
        }

        for (Obstacle o : obstacles) {
            System.out.println(o.getName() + " " + o.getMinX() + " " + o.getMaxX() + " " + o.getMinY() + " " + o.getMaxY());
            System.out.print("O_tL: ");
            System.out.print(convertObstacleArrayToString(o.get_tLObstacles()));
            System.out.print("O_tR: ");
            System.out.print(convertObstacleArrayToString(o.get_tRObstacles()));
            System.out.print("O_bL: ");
            System.out.print(convertObstacleArrayToString(o.get_bLObstacles()));
            System.out.print("O_bR: ");
            System.out.print(convertObstacleArrayToString(o.get_bRObstacles()));
            System.out.println();
            System.out.print("O_L: ");
            System.out.print(convertObstacleArrayToString(o.get_dLObstacles()));
            System.out.print("O_R: ");
            System.out.print(convertObstacleArrayToString(o.get_dRObstacles()));
            System.out.print("O_T: ");
            System.out.print(convertObstacleArrayToString(o.get_dTObstacles()));
            System.out.print("O_B: ");
            System.out.println(convertObstacleArrayToString(o.get_dBObstacles()));

        }

        /*
        Compute the shortest path between each corner of each pair obstacles
         */
        obstacleCornerToCornerDistCalculation(obstacles);

        /*
        Compute the shortest path between master/slave and each corner
         */
        System.out.println("HALLO");
        for (Obstacle o : obstacles){

            //master
            Map<PseudoBase, Integer> cornerMasterDistMin = new HashMap<>();
            Map<PseudoBase, Integer> cornerMasterDistXY = new HashMap<>();
            baseToObstacleDistCalculation(master, o, cornerMasterDistMin, cornerMasterDistXY);
            o.addTo_oBase_distMin(master, cornerMasterDistMin);
            o.addTo_oBase_distXY(master, cornerMasterDistXY);

            for (PseudoBase slave : slaves){
                Map<PseudoBase, Integer> cornerSlaveDistMin = new HashMap<>();
                Map<PseudoBase, Integer> cornerSlaveDistXY= new HashMap<>();
                baseToObstacleDistCalculation(slave, o, cornerSlaveDistMin, cornerSlaveDistXY);
                o.addTo_oBase_distMin(slave, cornerSlaveDistMin);
                o.addTo_oBase_distXY(slave, cornerSlaveDistXY);
            }


        }

        for (PseudoBase sv : slaves) {
            System.out.println(sv.getName() + "(" + sv.getX() + ", " + sv.getY() + ")");
            for (Obstacle o : obstacles) {
                System.out.print(o.getName() + "_dir:" + Arrays.toString(sv.getPseudo_oDir_qs().get(o)) + "|| ");
                System.out.println("_rel:" + Arrays.toString(sv.getPseudo_oRel_qs().get(o)) + "||");
            }
        }
        System.out.println("Master");
        for (Obstacle o : obstacles) {
            System.out.print(o.getName() + "_dir:" + Arrays.toString(master.getPseudo_oDir_qs().get(o)) + "|| ");
            System.out.println("_rel:" + Arrays.toString(master.getPseudo_oRel_qs().get(o)) + "||");
        }


        executor = new GurobiExecutor("LinearBusRouting_KO");
        executor.setMIPGap(0);
        executor.setMIPGapAbs(0);
//        executor.setTimeLimit(100);
        executor.setTimeLimit(7200);
//        executor.setPresolve(0);


        //build Gurobi Variables

        GurobiVariable busMin, busDiff, branchMin, branchDiff;
        busMin = new GurobiVariable(GRB.INTEGER, 0, M, "busLengthMin");
        executor.addVariable(busMin);
        busDiff = new GurobiVariable(GRB.INTEGER, 0, M, "busLengthDifference");
        executor.addVariable(busDiff);
        branchMin = new GurobiVariable(GRB.INTEGER, 0, M, "branchLengthMin");
        executor.addVariable(branchMin);
        branchDiff = new GurobiVariable(GRB.INTEGER, 0, M, "branchLengthDifference");
        executor.addVariable(branchDiff);

        ArrayList<VirtualPointVar> vps = new ArrayList<>();
        buildVars(obstacles, vps, slaves, master);
        executor.updateModelWithVars();
        System.out.println("#Variables = " + executor.getVariables().size());

        //build Constraints
        buildCons(obstacles, vps, slaves, master, busMin, busDiff, branchMin, branchDiff);


        executor.updateModelWithCons();
        System.out.println("#Constraints = " + executor.getConstraints().size());

        //Objective Function
        GurobiObjConstraint objCons = new GurobiObjConstraint();
        //todo Objective function

        objCons.addToLHS(busMin, Math.sqrt(2));
        objCons.addToLHS(busDiff, 1.0);
        objCons.addToLHS(branchMin, Math.sqrt(2));
        objCons.addToLHS(branchDiff, 1.0);

        objCons.setGoal(GRB.MINIMIZE);
        executor.setObjConstraint(objCons);

        //All Constraints
        /*for (GurobiConstraint c : executor.getConstraints()){
            System.out.println("C" + executor.getConstraints().indexOf(c));
            System.out.println(c);
        }*/

        executor.startOptimization();
        this.executor.getModel().write("debug.lp");
        int status = executor.getModel().get(GRB.IntAttr.Status);

        if (status == GRB.Status.UNBOUNDED) {
            System.out.println("The model cannot be solved " + "because it is unbounded");
        } else if (status == GRB.Status.OPTIMAL) {
            System.out.println("The optimal objective is " + executor.getModel().get(GRB.DoubleAttr.ObjVal));
        } else if (status != GRB.Status.INF_OR_UNBD && status != GRB.Status.INFEASIBLE) {
            System.out.println("Optimization was stopped with status " + status);
        } else if (executor.getModel().get(GRB.IntAttr.Status) == GRB.INFEASIBLE) {
            executor.whenInfeasibleWriteOutputTo("inf.ilp");
            System.out.println("HH:IIS");


//            // Do IIS
//            System.out.println("The model is infeasible; computing IIS");
//            LinkedList<String> removed = new LinkedList<String>();
//
//            // Loop until we reduce to a model that can be solved
//            while (true) {
//                executor.getModel().computeIIS();
//                System.out.println("\nThe following constraint cannot be satisfied:");
//                for (GRBConstr c : executor.getModel().getConstrs()) {
//                    if (c.get(GRB.IntAttr.IISConstr) == 1) {
//                        System.out.println(c.get(GRB.StringAttr.ConstrName));
//                        // Remove a single constraint from the model
//                        removed.add(c.get(GRB.StringAttr.ConstrName));
//                        executor.getModel().remove(c);
//                        break;
//                    }
//                }
//
//                System.out.println();
//                executor.getModel().optimize();
//                status = executor.getModel().get(GRB.IntAttr.Status);
//
//                if (status == GRB.Status.UNBOUNDED) {
//                    System.out.println("The model cannot be solved because it is unbounded");
//                    break;
//                }
//                if (status == GRB.Status.OPTIMAL) {
//                    break;
//                }
//                if (status != GRB.Status.INF_OR_UNBD &&
//                        status != GRB.Status.INFEASIBLE) {
//                    System.out.println("Optimization was stopped with status " +
//                            status);
//                    break;
//                }
//
//
//            }
//            System.out.println("\nThe following constraints were removed to get a feasible LP:");
//            for (String s : removed) {
//                System.out.print(s + " ");
//            }
//            System.out.println();
        }








        /*
        Retrieve
         */
        System.out.println("busMIN= " + busMin.getIntResult());
        System.out.println("busDIFF= " + busDiff.getIntResult());
        System.out.println("branchMIN= " + branchMin.getIntResult());
        System.out.println("brancDIFF=" + branchDiff.getIntResult());

//        for (int i = 0; i<vps.size(); ++i){
//            VirtualPointVar vp = vps.get(i);
//            System.out.print("v" + i + ":");
//            System.out.println("d_v->CoorS=" + convertGrbIntArrayToString(vp.vs_corDist_cqs));
//            for (PseudoBase sv : slaves){
//                System.out.print(sv.getName() + ":_q_ij=" +vp.vsCnn_q.get(sv).getIntResult() + "; ");
//                System.out.print("vs_dist:" + convertGrbIntArrayToString(vp.vs_dist_cqs.get(sv)) + "||");
//            }
//            System.out.println();
//        }
//        System.out.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

        for (int i = 0; i < vps.size(); ++i) {
            VirtualPointVar vp = vps.get(i);
            System.out.println("vp" + i + "=(" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ").");
            if (i == 0) {
                System.out.println("Master:");
                System.out.print("VM:detour_q:" + vp.vm_detour_q.getIntResult() + "|| ");
                System.out.println("VM_dist_cqs:" + convertGrbIntArrayToString(vp.vm_dist_cqs) + "|| ");
            }


            System.out.print("detour_q:" + vp.detour_q.getIntResult() + "|| ");
            System.out.println("dvv_cqs:" + convertGrbIntArrayToString(vp.dist_cqs) + "||");
            for (Obstacle o : obstacles) {
                System.out.print(o.getName() + " " + o.getMinX() + " " + o.getMaxX() + " " + o.getMinY() + " " + o.getMaxY() + "|| ");
                System.out.print("nonL:" + convertGrbIntArrayToString(vp.non_qs.get(o)) + "|| ");
                System.out.print("dir_qs:" + convertGrbIntArrayToString(vp.dir_qs.get(o)) + "|| ");
                System.out.print("rel_qs:" + convertGrbIntArrayToString(vp.rel_qs.get(o)) + "|| ");
                System.out.print("relD_qs:" + convertGrbIntArrayToString(vp.relD_qs.get(o)) + "|| ");
                int itmp = 0;
                for (Obstacle on : o.get_tLObstacles()) {
                    itmp += vp.rel_qs.get(on)[0].getIntResult();
                }
                System.out.print("_sum_tL:" + itmp + "|| ");
                itmp = 0;
                for (Obstacle on : o.get_tRObstacles()) {
                    itmp += vp.rel_qs.get(on)[1].getIntResult();
                }
                System.out.print("_sum_tR:" + itmp + "|| ");
                itmp = 0;
                for (Obstacle on : o.get_bLObstacles()) {
                    itmp += vp.rel_qs.get(on)[2].getIntResult();
                }
                System.out.print("_sum_bL:" + itmp + "|| ");
                itmp = 0;
                for (Obstacle on : o.get_bRObstacles()) {
                    itmp += vp.rel_qs.get(on)[3].getIntResult();
                }
                System.out.println("_sum_bR:" + itmp + "|| ");

            }
            if (vp.detour_q.getIntResult() == 1) {
                System.out.println("vpNext_Detour:");
                for (Obstacle om : obstacles) {
                    System.out.print(om.getName() + ":" + convertGrbIntArrayToString(vp.relObstacles_qs.get(om)));
                    System.out.print(" Dqs:" + convertGrbIntArrayToString(vp.relObstaclesD_qs.get(om)));
                    System.out.println(" ifRelevant: " + vp.relObstacles_q.get(om).getIntResult());
                }
                for (Obstacle om : obstacles) {
                    if (vp.inOutCnn_qs.get(om)[0].getIntResult() == 1) {
                        System.out.print("vp->: " + om.getName() + ":");
                        System.out.println("corner:" + convertGrbIntArrayToString(vp.corner_qs.get(om)) + "||");
                    }
                    if (vp.inOutCnn_qs.get(om)[1].getIntResult() == 1) {
                        System.out.print("vp<-:" + om.getName());
                        System.out.println("corner:" + convertGrbIntArrayToString(vp.corner_qs.get(om)) + "||");
                    }


                }
                for (Obstacle om : obstacles) {
                    for (Obstacle on : obstacles) {
                        if (vp.omOnCnn_q.get(om).get(on).getIntResult() == 1) {
                            System.out.print(om.getName() + "-> " + on.getName() + ":cnn" + "|| ");
                            System.out.print("cornerOm:" + convertGrbIntArrayToString(vp.corner_qs.get(om)) + "||cornerOn:" + convertGrbIntArrayToString(vp.corner_qs.get(on)) + "||");
                            System.out.println("dOmOn=" + convertGrbIntArrayToString(vp.dOmOn_cqs.get(om).get(on)) + "||");

                        }
                    }
                }
                System.out.println();

            }


            //System.out.println("vp" + i + "->Slave Relevant VVVVVVVVVVVVVVVVVVVVVVVVVV");
            for (PseudoBase sv : slaves) {
                if (vp.vsCnn_q.get(sv).getIntResult() == 1) {
                    System.out.println("vp" + i + "=(" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ")->" + sv.getName() + ":(" + sv.getX() + ", " + sv.getY() + ")");
                    System.out.print("d_v->CorrS= " + convertGrbContArrayToString(vp.vs_corDist_cqs) + "|| ");
                    System.out.println("|| d_vi_sj = " + convertGrbContArrayToString(vp.vs_dist_cqs.get(sv)) + "|| ");
                    double dtmp = Math.sqrt(2) * vp.aux_vsDist_iqs.get(sv)[2].getContResult() + vp.aux_vsDist_iqs.get(sv)[3].getContResult();
                    System.out.println("aux_vsDist_cqs=" + convertGrbContArrayToString(vp.aux_vsDist_iqs.get(sv)) + ": " + dtmp);
                    System.out.print("vs_detour_q:" + vp.vs_detour_q.get(sv).getIntResult() + "|| ");


                    System.out.println(sv.getName());
                    for (Obstacle om : obstacles) {
                        System.out.print(om.getName() + "_dir:" + Arrays.toString(sv.getPseudo_oDir_qs().get(om)) + "|| ");
                        System.out.print("_rel:" + Arrays.toString(sv.getPseudo_oRel_qs().get(om)) + "||");
                        int itmp = 0;
                        for (Obstacle on : om.get_tLObstacles()) {
                            itmp += sv.getPseudo_oRel_qs().get(on)[0];
                        }
                        System.out.print("_vs_sum_tL:" + itmp + "|| ");
                        itmp = 0;
                        for (Obstacle on : om.get_tRObstacles()) {
                            itmp += sv.getPseudo_oRel_qs().get(on)[1];
                        }
                        System.out.print("_vs_sum_tR:" + itmp + "|| ");
                        itmp = 0;
                        for (Obstacle on : om.get_bLObstacles()) {
                            itmp += sv.getPseudo_oRel_qs().get(on)[2];
                        }
                        System.out.print("_vs_sum_bL:" + itmp + "|| ");
                        itmp = 0;
                        for (Obstacle on : om.get_bRObstacles()) {
                            itmp += sv.getPseudo_oRel_qs().get(on)[3];
                        }
                        System.out.println("_vs_sum_bR:" + itmp + "|| ");

                    }

                    if (vp.vs_detour_q.get(sv).getIntResult() == 1) {
                        System.out.println("detour");
                        for (Obstacle om : obstacles) {
                            System.out.print(om.getName() + ":" + convertGrbIntArrayToString(vp.vs_relObstacles_qs.get(sv).get(om)));
                            System.out.print(" vs_Dqs:" + convertGrbIntArrayToString(vp.vs_relObstaclesD_qs.get(sv).get(om)));
                            System.out.println(" vs_ifRelevant: " + vp.vs_relObstacles_q.get(sv).get(om).getIntResult());
                        }
                        for (Obstacle om : obstacles) {
                            if (vp.vs_inOutCnn_qs.get(sv).get(om)[0].getIntResult() == 1) {
                                System.out.print("vs->: " + om.getName() + ":");
                                System.out.println("corner:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(om)) + "||");
                            }
                            if (vp.vs_inOutCnn_qs.get(sv).get(om)[1].getIntResult() == 1) {
                                System.out.print("vs<-:" + om.getName());
                                System.out.println("corner:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(om)) + "||");
                            }


                        }

                        for (Obstacle om : obstacles) {
                            for (Obstacle on : obstacles) {
                                if (vp.vs_omOnCnn_q.get(sv).get(om).get(on).getIntResult() == 1) {
                                    System.out.print(om.getName() + "-> " + on.getName() + ":cnn" + "|| ");
                                    System.out.print("cornerOm:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(om)) + "||cornerOn:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(on)) + "||");
                                    System.out.println("vs_dOmOn=" + convertGrbIntArrayToString(vp.vs_dOmOn_cqs.get(sv).get(om).get(on)));
                                }
                            }
                        }
                    }
                    System.out.println();
                }


            }
            System.out.println("vp" + i + "->Slave Relevant AAAAAAAAAAAAAAAAAAAAAAAAAA");

        }






        /*
        Debug
         */
        /*for (Obstacle o : obstacles) {
            System.out.println(o);
        }*/

        // Dispose of model and environment
//        executor.getModel().dispose();
//        executor.getEnv().dispose();

        return output;
    }

    private void baseToObstacleDistCalculation(PseudoBase base, Obstacle o, Map<PseudoBase, Integer> cornerBaseDistMin, Map<PseudoBase, Integer> cornerBaseDistXY) {
        for (PseudoBase corner : o.getBaseArray()){
            if (base.getPseudo_oRel_qs().get(o)[0] == 1){
                if (corner.equals(o.getLowerRight())){
                    int[] min_xy = comparePath(new ArrayList<>(Arrays.asList(base, o.getLowerLeft(), o.getLowerRight())), new ArrayList<>(Arrays.asList(base, o.getUpperRight(), o.getLowerRight())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }

            }else if (base.getPseudo_oRel_qs().get(o)[1] == 1){
                if (corner.equals(o.getLowerLeft())){
                    int[] min_xy = comparePath(new ArrayList<>(Arrays.asList(base, o.getUpperLeft(), o.getLowerLeft())), new ArrayList<>(Arrays.asList(base, o.getLowerRight(), o.getLowerLeft())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }

            }
            else if (base.getPseudo_oRel_qs().get(o)[2] == 1){
                if (corner.equals(o.getUpperRight())){
                    int[] min_xy = comparePath(new ArrayList<>(Arrays.asList(base, o.getUpperLeft(), o.getUpperRight())), new ArrayList<>(Arrays.asList(base, o.getLowerRight(), o.getUpperRight())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }

            }
            else if (base.getPseudo_oRel_qs().get(o)[3] == 1){
                if (corner.equals(o.getUpperLeft())){
                    int[] min_xy = comparePath(new ArrayList<>(Arrays.asList(base, o.getLowerLeft(), o.getUpperLeft())), new ArrayList<>(Arrays.asList(base, o.getUpperRight(), o.getUpperLeft())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }

            }
            else if (base.getPseudo_oRel_qs().get(o)[4] == 1){
                if (corner.equals(o.getUpperRight())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getUpperLeft(), o.getUpperRight())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else if (corner.equals(o.getLowerRight())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getLowerLeft(), o.getLowerRight())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }

            }
            else if (base.getPseudo_oRel_qs().get(o)[5] == 1){
                if (corner.equals(o.getUpperLeft())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getUpperRight(), o.getUpperLeft())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else if (corner.equals(o.getLowerLeft())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getLowerRight(), o.getLowerLeft())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }
            }
            else if (base.getPseudo_oRel_qs().get(o)[6] == 1){
                if (corner.equals(o.getLowerLeft())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getUpperLeft(), o.getLowerLeft())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else if (corner.equals(o.getLowerRight())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getUpperRight(), o.getLowerRight())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }
            }else if (base.getPseudo_oRel_qs().get(o)[7] == 1){
                if (corner.equals(o.getUpperLeft())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getLowerLeft(), o.getUpperLeft())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else if (corner.equals(o.getUpperRight())){
                    int[] min_xy = pathDist(new ArrayList<>(Arrays.asList(base, o.getLowerRight(), o.getUpperRight())));
                    cornerBaseDistMin.put(corner, min_xy[0]);
                    cornerBaseDistXY.put(corner, min_xy[1]);
                }else {
                    cornerBaseDistMin.put(corner, deltaMin(base, corner));
                    cornerBaseDistXY.put(corner, deltaXY(base, corner));
                }
            }else {
                cornerBaseDistMin.put(corner, deltaMin(base, corner));
                cornerBaseDistXY.put(corner, deltaXY(base, corner));
            }

        }
    }

    /**
     * Determine the distance between two corners of two different obstacles
     * @param obstacles ArrayList of all obstacles
     */
    private void obstacleCornerToCornerDistCalculation(ArrayList<Obstacle> obstacles) {
        for (Obstacle o : obstacles) {
            for (Obstacle on : obstacles) {
                if (!o.getName().equals(on.getName())) {
                    int[] distMin = new int[16];
                    int[] distXY = new int[16];

                    /*
                    0: ll->ll
                    1: ll->ul
                    2: ll->ur
                    3: ll->lr
                     */
                    PseudoBase startCorner = o.getLowerLeft();
                    //0: ll->ll
                    PseudoBase endCorner = on.getLowerLeft();
                    ArrayList<PseudoBase> bypassBases = new ArrayList<>();

                    int[] min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[0] = min_xy[0];
                    distXY[0] = min_xy[1];
                    //1: ll->ul
                    endCorner = on.getUpperLeft();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[1] = min_xy[0];
                    distXY[1] = min_xy[1];
                    //2: ll->ur
                    endCorner = on.getUpperRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[2] = min_xy[0];
                    distXY[2] = min_xy[1];
                    //3: ll->lr
                    endCorner = on.getLowerRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[3] = min_xy[0];
                    distXY[3] = min_xy[1];

                    /*
                    4: ul->ll
                    5: ul->ul
                    6: ul->ur
                    7: ul->lr
                     */
                    startCorner = o.getUpperLeft();
                    //4: ul->ll
                    endCorner = on.getLowerLeft();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[4] = min_xy[0];
                    distXY[4] = min_xy[1];
                    //5: ul->ul
                    endCorner = on.getUpperLeft();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[5] = min_xy[0];
                    distXY[5] = min_xy[1];
                    //6: ul->ur
                    endCorner = on.getUpperRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[6] = min_xy[0];
                    distXY[6] = min_xy[1];
                    //7: ul->lr
                    endCorner = on.getLowerRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[7] = min_xy[0];
                    distXY[7] = min_xy[1];

                    /*
                    8: ur->ll
                    9: ur->ul
                    10: ur->ur
                    11: ur->lr
                     */
                    startCorner = o.getUpperRight();
                    //8: ur->ll
                    endCorner = on.getLowerLeft();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[8] = min_xy[0];
                    distXY[8] = min_xy[1];
                    //9: ur->ul
                    endCorner = on.getUpperLeft();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[9] = min_xy[0];
                    distXY[9] = min_xy[1];
                    //10: ur->ur
                    endCorner = on.getUpperRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[10] = min_xy[0];
                    distXY[10] = min_xy[1];
                    //11: ur->lr
                    endCorner = on.getLowerRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[11] = min_xy[0];
                    distXY[11] = min_xy[1];

                    /*
                    12: lr->ll
                    13: lr->ul
                    14: lr->ur
                    15: lr->lr
                     */
                    startCorner = o.getLowerRight();
                    //12: lr->ll
                    endCorner = on.getLowerLeft();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[12] = min_xy[0];
                    distXY[12] = min_xy[1];
                    //13: lr->ul
                    endCorner = on.getUpperLeft();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[13] = min_xy[0];
                    distXY[13] = min_xy[1];
                    //14: lr->ur
                    endCorner = on.getUpperRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[14] = min_xy[0];
                    distXY[14] = min_xy[1];
                    //15: lr->lr
                    endCorner = on.getLowerRight();
                    bypassBases = new ArrayList<>();
                    min_xy = cornerDistanceCalculation(o, on, startCorner, endCorner, bypassBases);
                    distMin[15] = min_xy[0];
                    distXY[15] = min_xy[1];


                    o.addTo_oo_distMin(on, distMin);
                    o.addTo_oo_distXY(on, distXY);


                }
            }
        }
    }

    /**
     * Given two corner,
     * @param o obstacle that start point belongs to
     * @param on obstacle that end point belongs to
     * @param startCorner start point
     * @param endCorner end point
     * @param bypassBases give the path connecting from start point to end point
     * @return [distMin, distXY]
     */
    private int[] cornerDistanceCalculation(Obstacle o, Obstacle on, PseudoBase startCorner, PseudoBase endCorner, ArrayList<PseudoBase> bypassBases) {


        OppositeType type = null;
        int[] startO_qs = startCorner.getPseudo_oRel_qs().get(o);
        int[] endO_qs = endCorner.getPseudo_oRel_qs().get(o);
        int[] startOn_qs = startCorner.getPseudo_oRel_qs().get(on);
        int[] endOn_qs = endCorner.getPseudo_oRel_qs().get(on);


        //ul->lr
        boolean o_rel = false;
        boolean on_rel = false;
        if (startO_qs[0] + endO_qs[3] == 2) {
            o_rel = true;
            type = OppositeType.ul_lr;
        }
        if (startOn_qs[0] + endOn_qs[3] == 2) {
            on_rel = true;
            type = OppositeType.ul_lr;
        }
        if (startO_qs[0] == 1 && o.get_bRObstacles().contains(on) && endOn_qs[3] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.ul_lr;
        }
        if (type == OppositeType.ul_lr) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), endCorner)));


            } else if (!o_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, on.getUpperRight(), endCorner)));

            } else {
                return upperLeftLowerRight(o, on, startCorner, endCorner, bypassBases);
            }


        }


        //lr->ul
        if (startO_qs[3] + endO_qs[0] == 2) {
            o_rel = true;
            type = OppositeType.lr_ul;
        }
        if (startOn_qs[3] + endOn_qs[0] == 2) {
            on_rel = true;
            type = OppositeType.lr_ul;
        }
        if (startO_qs[3] == 1 && o.get_tLObstacles().contains(on) && endOn_qs[0] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.lr_ul;
        }
        if (type == OppositeType.lr_ul) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), endCorner)));

            } else if (!o_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, on.getUpperRight(), endCorner)));

            } else {
                return lowerRightUpperLeft(o, on, startCorner, endCorner, bypassBases);

            }

        }


        //ur->ll
        if (startO_qs[1] + endO_qs[2] == 2) {
            o_rel = true;
            type = OppositeType.ur_ll;
        }
        if (startOn_qs[1] + endOn_qs[2] == 2) {
            on_rel = true;
            type = OppositeType.ur_ll;
        }
        if (startO_qs[1] == 1 && o.get_bLObstacles().contains(on) && endOn_qs[2] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.ur_ll;
        }
        if (type == OppositeType.ur_ll) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), endCorner)));


            } else if (!o_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, on.getLowerRight(), endCorner)));

            } else {
                return upperRightLowerLeft(o, on, startCorner, endCorner, bypassBases);
            }


        }

        //ll->ur
        if (startO_qs[2] + endO_qs[1] == 2) {
            o_rel = true;
            type = OppositeType.ll_ur;
        }
        if (startOn_qs[2] + endOn_qs[1] == 2) {
            on_rel = true;
            type = OppositeType.ll_ur;
        }
        if (startO_qs[2] == 1 && o.get_tRObstacles().contains(on) && endOn_qs[1] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.ll_ur;
        }
        if (type == OppositeType.ll_ur) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), endCorner)));
            } else if (!o_rel) {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, on.getLowerRight(), endCorner)));
            } else {
                return lowerLeftUpperRight(o, on, startCorner, endCorner, bypassBases);
            }


        }


        //l->r
        if (startO_qs[4] + endO_qs[5] == 2) {
            o_rel = true;
            type = OppositeType.l_r;
        }
        if (startOn_qs[4] + endOn_qs[5] == 2) {
            on_rel = true;
            type = OppositeType.l_r;
        }
        if (startO_qs[4] == 1 && o.get_dRObstacles().contains(on) && endOn_qs[5] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.l_r;
        }
        if (type == OppositeType.l_r) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                if (startCorner.equals(o.getLowerLeft())) {
                    //start.o.ll
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), endCorner)));
                } else if (startCorner.equals(o.getUpperLeft())) {
                    //start.o.ul
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), endCorner)));
                } else {
                    System.out.println("l->r o_rel, but NOT o.ll or o.ul");
                }


            } else if (!o_rel) {
                if (endCorner.equals(on.getLowerRight())) {
                    //end.o.lr
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getLowerLeft(), endCorner)));
                } else if (endCorner.equals(on.getUpperRight())) {
                    //end.o.ur
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getUpperLeft(), endCorner)));
                } else {
                    System.out.println("l->r on_rel, but NOT on.lr or on.ur");
                }


            } else {
                if (startCorner.equals(o.getLowerLeft()) && endCorner.equals(on.getLowerRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMinY() <= on.getMinY() ? o.getLowerRight() : on.getLowerLeft(), endCorner)));
                } else if (startCorner.equals(o.getUpperLeft()) && endCorner.equals(on.getUpperRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMaxY() >= on.getMaxY() ? o.getUpperRight() : on.getUpperLeft(), endCorner)));

                } else if (startCorner.equals(o.getLowerLeft()) && endCorner.equals(on.getUpperRight())) {
                    return lowerLeftUpperRight(o, on, startCorner, endCorner, bypassBases);
                } else if (startCorner.equals(o.getUpperLeft()) && endCorner.equals(on.getLowerRight())) {
                    return upperLeftLowerRight(o, on, startCorner, endCorner, bypassBases);
                } else {
                    System.out.println("l->r, o_rel, on_rel: other Situation");
                }

            }

        }


        //r->l
        if (startO_qs[5] + endO_qs[4] == 2) {
            o_rel = true;
            type = OppositeType.r_l;
        }
        if (startOn_qs[5] + endOn_qs[4] == 2) {
            on_rel = true;
            type = OppositeType.r_l;
        }
        if (startO_qs[5] == 1 && o.get_dLObstacles().contains(on) && endOn_qs[4] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.r_l;
        }
        if (type == OppositeType.r_l) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                if (startCorner.equals(o.getLowerRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), endCorner)));
                } else if (startCorner.equals(o.getUpperRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), endCorner)));
                } else {
                    System.out.println("r->l, o_rel, but NOT o.lr or o.ur");
                }


            } else if (!o_rel) {
                if (endCorner.equals(on.getLowerLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getLowerRight(), endCorner)));
                } else if (endCorner.equals(on.getUpperLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getUpperRight(), endCorner)));
                } else {
                    System.out.println("r->l, on_rel, but NOT on.ll or on.ul");
                }

            } else {
                if (startCorner.equals(o.getLowerRight()) && endCorner.equals(on.getLowerLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMinY() <= on.getMinY() ? o.getLowerLeft() : on.getLowerRight(), endCorner)));

                } else if (startCorner.equals(o.getUpperRight()) && endCorner.equals(on.getUpperLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMaxY() >= on.getMaxY() ? o.getUpperLeft() : on.getUpperRight(), endCorner)));
                } else if (startCorner.equals(o.getLowerRight()) && endCorner.equals(on.getUpperLeft())) {
                    return lowerRightUpperLeft(o, on, startCorner, endCorner, bypassBases);
                } else if (startCorner.equals(o.getUpperRight()) && endCorner.equals(on.getLowerLeft())) {
                    return upperRightLowerLeft(o, on, startCorner, endCorner, bypassBases);
                } else {
                    System.out.println("r->l, o_rel, on_rel: other Situation");
                }
            }

        }


        //t->b
        if (startO_qs[6] + endO_qs[7] == 2) {
            o_rel = true;
            type = OppositeType.t_b;
        }
        if (startOn_qs[6] + endOn_qs[7] == 2) {
            on_rel = true;
            type = OppositeType.t_b;
        }
        if (startO_qs[6] == 1 && o.get_dBObstacles().contains(on) && endOn_qs[7] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.t_b;
        }
        if (type == OppositeType.t_b) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                if (startCorner.equals(o.getUpperLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), endCorner)));
                } else if (startCorner.equals(o.getUpperRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), endCorner)));
                } else {
                    System.out.println("t->b, o_rel, but NOT o.ul or o.ur");
                }

            } else if (!o_rel) {
                if (endCorner.equals(on.getLowerLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getUpperLeft(), endCorner)));
                } else if (endCorner.equals(on.getLowerRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getUpperRight(), endCorner)));
                } else {
                    System.out.println("t->b, on_rel, but NOT on.ll or on.lr");
                }

            } else {
                if (startCorner.equals(o.getUpperLeft()) && endCorner.equals(on.getLowerLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMinX() <= on.getMinX() ? o.getLowerLeft() : on.getUpperLeft(), endCorner)));
                } else if (startCorner.equals(o.getUpperRight()) && endCorner.equals(on.getLowerRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMaxX() >= on.getMaxX() ? o.getLowerRight() : on.getUpperRight(), endCorner)));
                } else if (startCorner.equals(o.getUpperLeft()) && endCorner.equals(on.getLowerRight())) {
                    return upperLeftLowerRight(o, on, startCorner, endCorner, bypassBases);
                } else if (startCorner.equals(o.getUpperRight()) && endCorner.equals(on.getLowerLeft())) {
                    return upperRightLowerLeft(o, on, startCorner, endCorner, bypassBases);
                } else {
                    System.out.println("t->b, o_rel, on_rel: other Situation");
                }

            }


        }

        //b->t
        if (startO_qs[7] + endO_qs[6] == 2) {
            o_rel = true;
            type = OppositeType.b_t;
        }
        if (startOn_qs[7] + endOn_qs[6] == 2) {
            on_rel = true;
            type = OppositeType.b_t;
        }
        if (startO_qs[7] == 1 && o.get_dTObstacles().contains(on) && endOn_qs[6] == 1) {
            o_rel = true;
            on_rel = true;
            type = OppositeType.b_t;
        }
        if (type == OppositeType.b_t) {
            System.out.println("detourType=" + type);
            if (o_rel && !on_rel) {
                if (startCorner.equals(o.getLowerLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), endCorner)));
                } else if (startCorner.equals(o.getLowerRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), endCorner)));
                } else {
                    System.out.println("b->t, o_rel, but NOT o.ll or o.lr");
                }

            } else if (!o_rel) {
                if (endCorner.equals(on.getUpperLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getLowerLeft(), endCorner)));
                } else if (endCorner.equals(on.getUpperRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, on.getLowerRight(), endCorner)));
                } else {
                    System.out.println("b->t, on_rel, but NOT on.ul or on.ur");
                }

            } else {
                if (startCorner.equals(o.getLowerLeft()) && endCorner.equals(on.getUpperLeft())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMinX() <= on.getMinX() ? o.getUpperLeft() : on.getLowerLeft(), endCorner)));

                } else if (startCorner.equals(o.getLowerRight()) && endCorner.equals(on.getUpperRight())) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getMaxX() >= on.getMaxX() ? o.getUpperRight() : on.getLowerRight(), endCorner)));
                } else if (startCorner.equals(o.getLowerLeft()) && endCorner.equals(on.getUpperRight())) {
                    return lowerLeftUpperRight(o, on, startCorner, endCorner, bypassBases);
                } else if (startCorner.equals(o.getLowerRight()) && endCorner.equals(on.getUpperLeft())) {
                    return lowerRightUpperLeft(o, on, startCorner, endCorner, bypassBases);
                } else {
                    System.out.println("b->t, o_rel, on_rel: other situation");
                }

            }

        }

        return new int[]{deltaMin(startCorner, endCorner), deltaXY(startCorner, endCorner)};
    }

    private int[] upperLeftLowerRight(Obstacle o, Obstacle on, PseudoBase startCorner, PseudoBase endCorner, ArrayList<PseudoBase> bypassBases) {
        if (o.getMaxY() >= on.getMaxY() && o.getMinY() >= on.getMinY()) {
            return compare3Paths(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getUpperRight(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getLowerLeft(), endCorner)));
        } else {

            if (o.getMaxY() < on.getMaxY()) {
                if (o.getMinY() <= on.getMinY()) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getLowerLeft(), endCorner)));
                } else {
                    return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getLowerLeft(), endCorner)));
                }
            } else {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getUpperRight(), endCorner)));
            }

        }
    }

    private int[] lowerRightUpperLeft(Obstacle o, Obstacle on, PseudoBase startCorner, PseudoBase endCorner, ArrayList<PseudoBase> bypassBases) {
        if (o.getMaxY() <= on.getMaxY() && o.getMinY() <= on.getMinY()) {
            return compare3Paths(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getUpperRight(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getUpperRight(), endCorner)));
        } else {
            if (o.getMaxY() > on.getMaxY()) {
                if (o.getMinY() >= o.getMinY()) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getUpperRight(), endCorner)));
                } else {
                    return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getUpperRight(), endCorner)));
                }
            } else {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerLeft(), on.getUpperRight(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperRight(), on.getUpperRight(), endCorner)));
            }

        }
    }

    private int[] upperRightLowerLeft(Obstacle o, Obstacle on, PseudoBase startCorner, PseudoBase endCorner, ArrayList<PseudoBase> bypassBases) {
        if (o.getMaxY() >= on.getMaxY() && o.getMinY() >= on.getMinY()) {
            return compare3Paths(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getLowerRight(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getLowerRight(), endCorner)));

        } else {
            if (o.getMaxY() < on.getMaxY()) {
                if (o.getMinY() <= on.getMinY()) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getLowerRight(), endCorner)));
                } else {
                    return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getLowerLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getLowerRight(), endCorner)));
                }

            } else {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getLowerRight(), endCorner)));

            }
        }
    }

    private int[] lowerLeftUpperRight(Obstacle o, Obstacle on, PseudoBase startCorner, PseudoBase endCorner, ArrayList<PseudoBase> bypassBases) {
        if (o.getMaxY() <= on.getMaxY() && o.getMinY() <= on.getMinY()) {
            return compare3Paths(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getLowerRight(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getUpperLeft(), endCorner)));
        } else {
            if (o.getMaxY() > on.getMaxY()) {
                if (o.getMinY() >= on.getMinY()) {
                    return pathDist(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getUpperLeft(), endCorner)));
                } else {
                    return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getLowerRight(), endCorner)));
                }


            } else {
                return comparePath(bypassBases, new ArrayList<>(Arrays.asList(startCorner, o.getUpperLeft(), on.getUpperLeft(), endCorner)), new ArrayList<>(Arrays.asList(startCorner, o.getLowerRight(), on.getUpperLeft(), endCorner)));

            }
        }
    }

    //todo: compare ArrayList of paths.
    private int[] compare3Paths(ArrayList<PseudoBase> bypassBases, ArrayList<PseudoBase> path1, ArrayList<PseudoBase> path2, ArrayList<PseudoBase> path3) {
        int path1Min = 0, path1XY = 0, path2Min = 0, path2XY = 0, path3Min = 0, path3XY = 0;
        for (int i = 0; i < path1.size() - 1; ++i) {
            path1Min += deltaMin(path1.get(i), path1.get(i + 1));
            path1XY += deltaXY(path1.get(i), path1.get(i + 1));
        }
        for (int i = 0; i < path2.size() - 1; ++i) {
            path2Min += deltaMin(path2.get(i), path2.get(i + 1));
            path2XY += deltaXY(path2.get(i), path2.get(i + 1));
        }
        for (int i = 0; i < path3.size() - 1; ++i) {
            path3Min += deltaMin(path3.get(i), path3.get(i + 1));
            path3XY += deltaXY(path3.get(i), path3.get(i + 1));
        }

        ArrayList<Double> tmpArray = new ArrayList<>(Arrays.asList(octDist(path1Min, path1XY), octDist(path2Min, path2XY),octDist(path3Min, path3XY)));
        double tmp = Collections.min(tmpArray);
        if (tmp == octDist(path1Min, path1XY)){
            bypassBases.addAll(path1);
            for (PseudoBase base : bypassBases){
                System.out.print(base.getName() + "||");
            }
            System.out.println(path1Min + " " + path1XY);
            return new int[]{path1Min, path1XY};
        }else if (tmp == octDist(path2Min, path2XY)){
            bypassBases.addAll(path2);
            for (PseudoBase base : bypassBases){
                System.out.print(base.getName() + "||");
            }
            System.out.println(path2Min + " " + path2XY);
            return new int[]{path2Min, path2XY};
        }
        else if (tmp == octDist(path3Min, path3XY)){
            bypassBases.addAll(path3);
            for (PseudoBase base : bypassBases){
                System.out.print(base.getName() + "||");
            }
            System.out.println(path3Min + " " + path3XY);
            return new int[]{path3Min, path3XY};
        }else {
            System.out.println("path wrong");
            return new int[]{M, M};
        }
    }

    private int[] comparePath(ArrayList<PseudoBase> bypassBases, ArrayList<PseudoBase> path1, ArrayList<PseudoBase> path2) {
        int path1Min = 0, path1XY = 0, path2Min = 0, path2XY = 0;
        for (int i = 0; i < path1.size() - 1; ++i) {
            path1Min += deltaMin(path1.get(i), path1.get(i + 1));
            path1XY += deltaXY(path1.get(i), path1.get(i + 1));
        }
        for (int i = 0; i < path2.size() - 1; ++i) {
            path2Min += deltaMin(path2.get(i), path2.get(i + 1));
            path2XY += deltaXY(path2.get(i), path2.get(i + 1));
        }
        if (octDist(path1Min, path1XY) <= octDist(path2Min, path2XY)) {
            bypassBases.addAll(path1);
            for (PseudoBase base : bypassBases){
                System.out.print(base.getName() + "||");
            }
            System.out.println(path1Min + " " + path1XY);
            return new int[]{path1Min, path1XY};
        } else {
            bypassBases.addAll(path2);
            for (PseudoBase base : bypassBases){
                System.out.print(base.getName() + "||");
            }
            System.out.println(path2Min + " " + path2XY);
            return new int[]{path2Min, path2XY};
        }
    }

    private int[] comparePath(ArrayList<PseudoBase> path1, ArrayList<PseudoBase> path2) {
        int path1Min = 0, path1XY = 0, path2Min = 0, path2XY = 0;
        for (int i = 0; i < path1.size() - 1; ++i) {
            path1Min += deltaMin(path1.get(i), path1.get(i + 1));
            path1XY += deltaXY(path1.get(i), path1.get(i + 1));
        }
        for (int i = 0; i < path2.size() - 1; ++i) {
            path2Min += deltaMin(path2.get(i), path2.get(i + 1));
            path2XY += deltaXY(path2.get(i), path2.get(i + 1));
        }
        if (octDist(path1Min, path1XY) <= octDist(path2Min, path2XY)) {
            System.out.println(path1Min + " " + path1XY);
            return new int[]{path1Min, path1XY};
        } else {
            System.out.println(path2Min + " " + path2XY);
            return new int[]{path2Min, path2XY};
        }
    }

    private int[] pathDist(ArrayList<PseudoBase> bypassBases, ArrayList<PseudoBase> path) {
        bypassBases.addAll(path);
        for (PseudoBase base : bypassBases){
            System.out.print(base.getName() + "||");
        }
        int pathMin = 0, pathXY = 0;
        for (int i = 0; i < path.size() - 1; ++i) {
            pathMin += deltaMin(path.get(i), path.get(i + 1));
            pathXY += deltaXY(path.get(i), path.get(i + 1));
        }
        System.out.println(pathMin + " " + pathXY);
        return new int[]{pathMin, pathXY};
    }

    private int[] pathDist(ArrayList<PseudoBase> path) {
        int pathMin = 0, pathXY = 0;
        for (int i = 0; i < path.size() - 1; ++i) {
            pathMin += deltaMin(path.get(i), path.get(i + 1));
            pathXY += deltaXY(path.get(i), path.get(i + 1));
        }
        System.out.println(pathMin + " " + pathXY);
        return new int[]{pathMin, pathXY};
    }


    private double octDist(int pmin, int pxy) {
        return Math.sqrt(2) * pmin + pxy;
    }

    public int deltaMin(PseudoBase base1, PseudoBase base2) {
        return Math.min(Math.abs(base1.getX() - base2.getX()), Math.abs(base1.getY() - base2.getY()));
    }

    public int deltaXY(PseudoBase base1, PseudoBase base2) {
        return Math.abs(Math.abs(base1.getX() - base2.getX()) - Math.abs(base1.getY() - base2.getY()));
    }

    public void buildCons(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiVariable busMin, GurobiVariable busXY, GurobiVariable branchMin, GurobiVariable branchDiff) throws GRBException {
        double minDist = 1;

        GurobiConstraint c;


        VirtualPointVar vpN;


        //busLength Min
        GurobiConstraint c_busMin = new GurobiConstraint();
        c_busMin.setName("c_busMin");
        c_busMin.addToLHS(busMin, 1.0);
        c_busMin.setSense('=');
        executor.addConstraint(c_busMin);
        //busLength Difference
        GurobiConstraint c_busXY = new GurobiConstraint();
        c_busXY.setName("c_busXY");
        c_busXY.addToLHS(busXY, 1.0);
        c_busXY.setSense('=');
        executor.addConstraint(c_busXY);
        //branchLength Min
        GurobiConstraint c_branchMin = new GurobiConstraint();
        c_branchMin.setName("c_branchMin");
        c_branchMin.addToLHS(branchMin, 1.0);
        c_branchMin.setSense('=');
        executor.addConstraint(c_branchMin);
        //branchLength Difference
        GurobiConstraint c_branchXY = new GurobiConstraint();
        c_branchXY.setName("c_branchXY");
        c_branchXY.addToLHS(branchDiff, 1.0);
        c_branchXY.setSense('=');
        executor.addConstraint(c_branchXY);


        //vp->Slave connection
        for (PseudoBase sv : slaves) {
            c = new GurobiConstraint();
            for (VirtualPointVar vp : virtualPointVars) {
                c.addToLHS(vp.vsCnn_q.get(sv), 1.0);
            }
            c.setSense('=');
            c.setRHSConstant(1.0);
            executor.addConstraint(c);
        }

        //Master Relevant
        buildCons_Master(obstacles, master, c_busMin, c_busXY, virtualPointVars.get(0), virtualPointVars);

        for (VirtualPointVar vp : virtualPointVars) {

            /*
            non-overlapping and opposite relations recognition
             */
            for (Obstacle o : obstacles) {
                buildCons_nonOverlapAndOppositeRelation(minDist, vp, o);
            }


            /*
            Index 0 -- vps.size() - 1:
            Connection with next virtualPoint
             */
            if (virtualPointVars.indexOf(vp) < virtualPointVars.size() - 1) {
                vpN = virtualPointVars.get(virtualPointVars.indexOf(vp) + 1);


                //add To bus length:
                c_busMin.addToRHS(vp.dist_cqs[0], 1.0);
                c_busXY.addToRHS(vp.dist_cqs[1], 1.0);

                /*
                d_ij without Detour
                VVVV
                 */
                c = new GurobiConstraint();
                c.setName("pathLength_min");
                c.addToLHS(vp.dist_cqs[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[2], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("pathLength_xy");
                c.addToLHS(vp.dist_cqs[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[3], 1.0);
                executor.addConstraint(c);

                auxiliaryDistCons_VarVar(vp.x, vpN.x, vp.y, vpN.y, "d_ij_woDetour" + virtualPointVars.indexOf(vp), vp.aux_dist_iqs, vp.auxQ_dist);

                /*
                AAAA
                d_ij
                 */

                //d_ij regarding Detour
                GurobiConstraint c_pathLengthMin = new GurobiConstraint();
                c_pathLengthMin.setName("pathLength_detour_min");
                c_pathLengthMin.addToLHS(vp.dist_cqs[0], 1.0);
                c_pathLengthMin.setSense('>');
                c_pathLengthMin.addToRHS(vp.dOut_cqs[0], 1.0);
                c_pathLengthMin.addToRHS(vp.dIn_cqs[0], 1.0);
                c_pathLengthMin.addToRHS(vp.detour_q, M);
                c_pathLengthMin.setRHSConstant(-M);
                executor.addConstraint(c_pathLengthMin);
                GurobiConstraint c_pathLengthXY = new GurobiConstraint();
                c_pathLengthXY.setName("pathLength_detour_xy");
                c_pathLengthXY.addToLHS(vp.dist_cqs[1], 1.0);
                c_pathLengthXY.setSense('>');
                c_pathLengthXY.addToRHS(vp.dOut_cqs[1], 1.0);
                c_pathLengthXY.addToRHS(vp.dIn_cqs[1], 1.0);
                c_pathLengthXY.addToRHS(vp.detour_q, M);
                c_pathLengthXY.setRHSConstant(-M);
                executor.addConstraint(c_pathLengthXY);




                //detour_triggering_aux.2: if one obstacle is relevant => detour trigger
                GurobiConstraint c_detour_geq = new GurobiConstraint();
                c_detour_geq.setName("detourTriggerAux2_geq");
                c_detour_geq.addToLHS(vp.detour_q, obstacles.size());
                c_detour_geq.setSense('>');
                executor.addConstraint(c_detour_geq);
                GurobiConstraint c_detour_leq = new GurobiConstraint();
                c_detour_leq.setName("detourTriggerAux2_leq");
                c_detour_leq.addToLHS(vp.detour_q, 1.0);
                c_detour_leq.setSense('<');
                executor.addConstraint(c_detour_leq);
                //cnnRules.5: if detour => startPoint should connect one of the relevant obstacle
                GurobiConstraint c_svpToObstacle = new GurobiConstraint();
                c_svpToObstacle.setName("cnnRules.5");
                c_svpToObstacle.setSense('=');
                c_svpToObstacle.addToRHS(vp.detour_q, 1.0);
                executor.addConstraint(c_svpToObstacle);
                //cnnRules.6: if detour => one of the relevant obstacle should connect to endPoint
                GurobiConstraint c_obstacleToEvp = new GurobiConstraint();
                c_obstacleToEvp.setName("cnnRules.6");
                c_obstacleToEvp.setSense('=');
                c_obstacleToEvp.addToRHS(vp.detour_q, 1.0);
                executor.addConstraint(c_obstacleToEvp);

                for (Obstacle o : obstacles) {
                    c_detour_geq.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    c_detour_leq.addToRHS(vp.relObstacles_q.get(o), 1.0);

                    //cnnRules.5
                    //c_svpToObstacle.addToLHS(vp.inOutCnn_qs.get(o)[0], vp.relObstacles_q.get(o), 1.0);
                    //cnnRules.5_lin
                    c = new GurobiConstraint();
                    c.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.inOutCnn_qs.get(o)[0], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.inOutCnn_qs.get(o)[0], 1.0);
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);
                    c_svpToObstacle.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[0], 1.0);


                    //cnnRules.6
                    //c_obstacleToEvp.addToLHS(vp.inOutCnn_qs.get(o)[1], vp.relObstacles_q.get(o), 1.0);
                    //cnnRules.6_lin
                    c = new GurobiConstraint();
                    c.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.inOutCnn_qs.get(o)[1], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.inOutCnn_qs.get(o)[1], 1.0);
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);
                    c_obstacleToEvp.addToLHS(vp.auxQuad_inOutCorners_qs.get(o)[1], 1.0);



                    //(1)ul->lr
                    //ul->lr.2_o
                    c = new GurobiConstraint();
                    c.setName("ul->lr.2_o");
                    c.addToLHS(vp.relObstacles_qs.get(o)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_bRObstacles()) {
                        //ul->lr.1_o
                        c = new GurobiConstraint();
                        c.setName("ul->lr.1_o");
                        c.addToLHS(vp.relObstacles_qs.get(o)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
                        c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);


                        if (!o.getName().equals(on.getName())) {
                            //ul->lr.1_on
                            c = new GurobiConstraint();
                            c.setName("ul->lr.1_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[0], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);
                            //ul->lr.2_on
                            c = new GurobiConstraint();
                            c.setName("ul->lr.2_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[0], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                            executor.addConstraint(c);
                        }
                    }
                    //(2)lr->ul
                    c = new GurobiConstraint();
                    c.setName("lr->ul.2_o");
                    c.addToLHS(vp.relObstacles_qs.get(o)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_tLObstacles()) {
                        //lr->ul.1_o
                        c = new GurobiConstraint();
                        c.setName("lr->ul.1_o");
                        c.addToLHS(vp.relObstacles_qs.get(o)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
                        c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        if (!o.getName().equals(on.getName())) {
                            //lr->ul.1_on
                            c = new GurobiConstraint();
                            c.setName("lr->ul.1_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[1], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);

                            //lr->ul.2_on
                            c = new GurobiConstraint();
                            c.setName("lr->ul.2_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[1], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                            executor.addConstraint(c);

                        }
                    }
                    //(3)ur->ll
                    c = new GurobiConstraint();
                    c.setName("ur->ll.2_o");
                    c.addToLHS(vp.relObstacles_qs.get(o)[2], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_bLObstacles()) {
                        //ur->ll.1_o
                        c = new GurobiConstraint();
                        c.setName("ur->ll.1_o");
                        c.addToLHS(vp.relObstacles_qs.get(o)[2], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
                        c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);


                        if (!o.getName().equals(on.getName())) {
                            //ur->ll.1_on
                            c = new GurobiConstraint();
                            c.setName("ur->ll.1_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);

                            //ur->lr.2_on
                            c = new GurobiConstraint();
                            c.setName("ur->ll.2_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[2], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                            executor.addConstraint(c);
                        }
                    }
                    //(4)ll->ur
                    c = new GurobiConstraint();
                    c.setName("ll->ur.2_o");
                    c.addToLHS(vp.relObstacles_qs.get(o)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_tRObstacles()) {
                        //ll->ur.1_o
                        c = new GurobiConstraint();
                        c.setName("ll->ur.1_o");
                        c.addToLHS(vp.relObstacles_qs.get(o)[3], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
                        c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        if (!o.getName().equals(on.getName())) {
                            //ll->ur.1_on
                            c = new GurobiConstraint();
                            c.setName("ll->ur.1_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[3], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);

                            //ll->ur.2_on
                            c = new GurobiConstraint();
                            c.setName("ll->ur.2_on");
                            c.addToLHS(vp.relObstacles_qs.get(on)[3], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
                            c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                            executor.addConstraint(c);
                        }
                    }
                    //(1)l->r
                    c = new GurobiConstraint();
                    c.setName("l->r.2_o");
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_dRObstacles()) {
                        //(1)l->r.1_o
                        c = new GurobiConstraint();
                        c.setName("l->r.1_o");
                        c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.relD_qs.get(on)[1], 1.0);
                        c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        if (!o.getName().equals(on.getName())) {
                            //(1)l->r.1_on
                            c = new GurobiConstraint();
                            c.setName("l->r.1_on");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[0], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.relD_qs.get(on)[1], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);

                            //(1)l->r.2_on
                            c = new GurobiConstraint();
                            c.setName("l->r.2_on");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[0], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.relD_qs.get(on)[1], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                            executor.addConstraint(c);

                        }


                    }
                    //(2)r->l
                    c = new GurobiConstraint();
                    c.setName("r->l.2_o");
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_dLObstacles()) {
                        //(2)r->l.1_o
                        c = new GurobiConstraint();
                        c.setName("r->l.1_o");
                        c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.relD_qs.get(on)[0], 1.0);
                        c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);
                        if (!o.getName().equals(on.getName())) {

                            //(2)r->l.1_on
                            c = new GurobiConstraint();
                            c.setName("r->l.1_o");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[1], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.relD_qs.get(on)[0], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);

                            //(2)r->l.2_on
                            c = new GurobiConstraint();
                            c.setName("r->l.2_on");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[1], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.relD_qs.get(on)[0], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                            executor.addConstraint(c);

                        }


                    }
                    //(3)t->b
                    c = new GurobiConstraint();
                    c.setName("t->b.2_o");
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_dBObstacles()) {
                        //(3)t->b.1_o
                        c = new GurobiConstraint();
                        c.setName("t->b.1_o");
                        c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.relD_qs.get(on)[3], 1.0);
                        c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);
                        if (!o.getName().equals(on.getName())) {

                            //(3)t->b.1_on
                            c = new GurobiConstraint();
                            c.setName("t->b.1_on");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.relD_qs.get(on)[3], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);

                            //(3)t->b.2_on
                            c = new GurobiConstraint();
                            c.setName("t->b.2_on");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[2], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.relD_qs.get(on)[3], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                            executor.addConstraint(c);

                        }


                    }
                    //(4)b->t
                    c = new GurobiConstraint();
                    c.setName("b->t.2_o");
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : o.get_dTObstacles()) {
                        //(4) b->t.1_o
                        c = new GurobiConstraint();
                        c.setName("b->t.1_o");
                        c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.relD_qs.get(on)[2], 1.0);
                        c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        if (!o.getName().equals(on.getName())) {
                            //(4) b->t.1_on
                            c = new GurobiConstraint();
                            c.setName("b->t.1_on");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[3], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.relD_qs.get(on)[2], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);

                            //(4) b->t.2_on
                            c = new GurobiConstraint();
                            c.setName("b->t.2_on");
                            c.addToLHS(vp.relObstaclesD_qs.get(on)[3], 2.0);
                            c.setSense('<');
                            c.addToRHS(vpN.relD_qs.get(on)[2], 1.0);
                            c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                            executor.addConstraint(c);

                        }


                    }


                    //Opposite Relation Recognition: aux.1
                    c = new GurobiConstraint();
                    c.setName("aux.1");
                    c.addToLHS(vp.relObstacles_qs.get(o)[0], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(o)[1], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(o)[2], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(o)[3], 1.0);
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
                    c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relObstacles_q.get(o), 8.0);
                    executor.addConstraint(c);

                    /*
                    Connection Rules:
                     */
                    //Corner
                    c = new GurobiConstraint();
                    c.setName("corner");
                    c.addToLHS(vp.corner_qs.get(o)[0], 1.0);
                    c.addToLHS(vp.corner_qs.get(o)[1], 1.0);
                    c.addToLHS(vp.corner_qs.get(o)[2], 1.0);
                    c.addToLHS(vp.corner_qs.get(o)[3], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    executor.addConstraint(c);

                    //outCorner: vi -> o
                    buildCons_inOutCornerSelection(vp.inOutCnn_qs.get(o)[0], vp.outCorner_qs.get(o), vp.rel_qs.get(o), vp.relD_qs.get(o), "v" + virtualPointVars.indexOf(vp) + "->o.Corner");
                    //inCorner: o -> vi
                    buildCons_inOutCornerSelection(vp.inOutCnn_qs.get(o)[1], vp.inCorner_qs.get(o), vpN.rel_qs.get(o), vpN.relD_qs.get(o), "o.Corner->vNxt" + virtualPointVars.indexOf(vp));




                    //cnnRules.1: q_om->om = 0
                    c = new GurobiConstraint();
                    c.setName("cnnRules.1");
                    c.addToLHS(vp.omOnCnn_q.get(o).get(o), 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);

                    //cnnRules.3: om relevant => connect to one relevant on or endpoint
                    GurobiConstraint c_obstacleOutCnn = new GurobiConstraint();
                    c_obstacleOutCnn.setName("cnnRules.3");
                    c_obstacleOutCnn.addToLHS(vp.relObstacles_q.get(o), 1.0);
                    c_obstacleOutCnn.setSense('=');
                    c_obstacleOutCnn.addToRHS(vp.inOutCnn_qs.get(o)[1], 1.0);
                    executor.addConstraint(c_obstacleOutCnn);
                    //cnnRules.4: om relevant => be connected by one relevant on or startpoint
                    GurobiConstraint c_obstacleInCnn = new GurobiConstraint();
                    c_obstacleInCnn.setName("cnnRules.4");
                    c_obstacleInCnn.addToLHS(vp.relObstacles_q.get(o), 1.0);
                    c_obstacleInCnn.setSense('=');
                    c_obstacleInCnn.addToRHS(vp.inOutCnn_qs.get(o)[0], 1.0);
                    executor.addConstraint(c_obstacleInCnn);


                    //Om ----> On
                    for (Obstacle on : obstacles) {
                        if (!o.getName().equals(on.getName())) {

                            //cornerCnn: q_om->on = sum q_om.corner->on.corner
                            c = new GurobiConstraint();
                            c.setName("cornerCnn");
                            c.addToLHS(vp.omOnCnn_q.get(o).get(on), 1.0);
                            c.setSense('=');
                            for (int i = 0; i < vp.omOnCorner_qs.get(o).get(on).length; ++i){
                                c.addToRHS(vp.omOnCorner_qs.get(o).get(on)[i], 1.0);
                            }
                            executor.addConstraint(c);

                            //cnnRules.2: q_om->on + q_on->om <= 1
                            c = new GurobiConstraint();
                            c.setName("cnnRules.2");
                            c.addToLHS(vp.omOnCnn_q.get(o).get(on), 1.0);
                            c.addToLHS(vp.omOnCnn_q.get(on).get(o), 1.0);
                            c.setSense('<');
                            c.setRHSConstant(1.0);
                            executor.addConstraint(c);

                            //cnnRules.3
                            //c_obstacleOutCnn.addToRHS(vp.omOnCnn_q.get(o).get(on), vp.relObstacles_q.get(on), 1.0);
                            //cnnRules.3_lin
                            c = new GurobiConstraint();
                            c.addToLHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[0], 1.0);
                            c.setSense('<');
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), 1.0);
                            executor.addConstraint(c);
                            c = new GurobiConstraint();
                            c.addToLHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[0], 1.0);
                            c.setSense('<');
                            c.addToRHS(vp.relObstacles_q.get(on), 1.0);
                            executor.addConstraint(c);
                            c = new GurobiConstraint();
                            c.addToLHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[0], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), 1.0);
                            c.addToRHS(vp.relObstacles_q.get(on), 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);
                            c_obstacleOutCnn.addToRHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[0], 1.0);

                            //cnnRules.4
                            //c_obstacleInCnn.addToRHS(vp.omOnCnn_q.get(on).get(o), vp.relObstacles_q.get(o), 1.0);
                            //cnnRules.4_lin
                            c = new GurobiConstraint();
                            c.addToLHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[1], 1.0);
                            c.setSense('<');
                            c.addToRHS(vp.omOnCnn_q.get(on).get(o), 1.0);
                            executor.addConstraint(c);
                            c = new GurobiConstraint();
                            c.addToLHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[1], 1.0);
                            c.setSense('<');
                            c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                            executor.addConstraint(c);
                            c = new GurobiConstraint();
                            c.addToLHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[1], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.omOnCnn_q.get(on).get(o), 1.0);
                            c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);
                            c_obstacleInCnn.addToRHS(vp.auxQuad_omOnCorner_qs.get(o).get(on)[1], 1.0);

                            for (int cnt = 0; cnt < vp.omOnCorner_qs.get(o).get(on).length; ++cnt){
                                //pathLength.2:Min
                                c = new GurobiConstraint();
                                c.setName("pathLength.2:Min");
                                c.addToLHS(vp.dOmOn_cqs.get(o).get(on)[0], 1.0);
                                c.setSense('>');
                                c.addToRHS(vp.omOnCorner_qs.get(o).get(on)[cnt], M);
                                c.setRHSConstant(o.getOo_distMin().get(on)[cnt] - M);
                                executor.addConstraint(c);
                                //pathLength.2:XY
                                c = new GurobiConstraint();
                                c.setName("pathLength.2:XY");
                                c.addToLHS(vp.dOmOn_cqs.get(o).get(on)[1], 1.0);
                                c.setSense('>');
                                c.addToRHS(vp.omOnCorner_qs.get(o).get(on)[cnt], M);
                                c.setRHSConstant(o.getOo_distXY().get(on)[cnt] - M);
                                executor.addConstraint(c);
                            }



                            //todo:d_ij:pl.m->n_MIN<= M
                            c = new GurobiConstraint();
                            c.setName("d_ij:pl.m->n_MIN<=M");
                            c.addToLHS(vp.dOmOn_cqs.get(o).get(on)[0], 1.0);
                            c.setSense('<');
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), M);
                            executor.addConstraint(c);

                            c_pathLengthMin.addToRHS(vp.dOmOn_cqs.get(o).get(on)[0], 1.0);


                            //todo: d_ij:pl.m->n_DIFF<=M
                            c = new GurobiConstraint();
                            c.setName("d_ij:pl.m->n_DIFF<=M");
                            c.addToLHS(vp.dOmOn_cqs.get(o).get(on)[1], 1.0);
                            c.setSense('<');
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), M);
                            executor.addConstraint(c);


                            c_pathLengthXY.addToRHS(vp.dOmOn_cqs.get(o).get(on)[1], 1.0);


                        }
                    }


                    /*
                    dvi-> and d->v_i+1: pathLength.1, pathLength.3
                     */
                    for (int cnt = 0; cnt < o.getBaseArray().size(); ++cnt){
                        PseudoBase corner = o.getBaseArray().get(cnt);
                        //pathLength.1:Min
                        c = new GurobiConstraint();
                        c.setName("pathLength.1:Min");
                        c.addToLHS(vp.dOut_cqs[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.aux_dOutCorner_iqs.get(o).get(corner)[2], 1.0);
                        c.addToRHS(vp.outCorner_qs.get(o)[cnt], M);
                        c.setRHSConstant(-M);
                        executor.addConstraint(c);
                        //pathLength.1:XY
                        c = new GurobiConstraint();
                        c.setName("pathLength.1:XY");
                        c.addToLHS(vp.dOut_cqs[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.aux_dOutCorner_iqs.get(o).get(corner)[3], 1.0);
                        c.addToRHS(vp.outCorner_qs.get(o)[cnt], M);
                        c.setRHSConstant(-M);
                        executor.addConstraint(c);

                        auxiliaryDistCons_VarBase(vp.x, vp.y, corner, "dv_i->", vp.aux_dOutCorner_iqs.get(o).get(corner), vp.auxQ_dOutCorner.get(o).get(corner));

                        //pathLength.3:Min
                        c = new GurobiConstraint();
                        c.setName("pathLength.3:Min");
                        c.addToLHS(vp.dIn_cqs[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.aux_dInCorner_iqs.get(o).get(corner)[2], 1.0);
                        c.addToRHS(vp.inCorner_qs.get(o)[cnt], M);
                        c.setRHSConstant(-M);
                        executor.addConstraint(c);
                        //pathLength.3:XY
                        c = new GurobiConstraint();
                        c.setName("pathLength.3:XY");
                        c.addToLHS(vp.dIn_cqs[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.aux_dInCorner_iqs.get(o).get(corner)[3], 1.0);
                        c.addToRHS(vp.inCorner_qs.get(o)[cnt], M);
                        c.setRHSConstant(-M);
                        executor.addConstraint(c);

                        auxiliaryDistCons_VarBase(vpN.x, vpN.y, corner, "d->v_i+1", vp.aux_dInCorner_iqs.get(o).get(corner), vp.auxQ_dInCorner.get(o).get(corner));



                    }




                }


            }



            /*
            Connection with Slaves
             */
            GurobiConstraint c_vsCnn = new GurobiConstraint();
            c_vsCnn.setSense('=');
            c_vsCnn.setRHSConstant(1.0);
            executor.addConstraint(c_vsCnn);

            for (PseudoBase sv : slaves) {
                //sv.1
                c_vsCnn.addToLHS(vp.vsCnn_q.get(sv), 1.0);

                //add to branch length:
                c_branchMin.addToRHS(vp.vs_corDist_cqs[0], 1.0);
                c_branchXY.addToRHS(vp.vs_corDist_cqs[1], 1.0);

                /*
                sv.3: d_vi_s^i = d_vi_sj
                d_vi_s^i >= d_vi_sj - (1 - q_vi_sj) * M
                d_vi_s^i <= d_vi_sj + (1 - q_vi_sj) * M
                Distance between vi and its corresponding slave = distance between vi and sj if vi and sj are connected
                 */
                //sv.3_geq
                c = new GurobiConstraint();
                c.setName("sv.3_geq:Min");
                c.addToLHS(vp.vs_corDist_cqs[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.vs_dist_cqs.get(sv)[0], 1.0);
                c.addToRHS(vp.vsCnn_q.get(sv), M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("sv.3_geq:XY");
                c.addToLHS(vp.vs_corDist_cqs[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.vs_dist_cqs.get(sv)[1], 1.0);
                c.addToRHS(vp.vsCnn_q.get(sv), M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                //sv.3_leq
//                c = new GurobiConstraint();
//                c.setName("sv.3_leq:Min");
//                c.addToLHS(vp.vs_corDist_cqs[0], 1.0);
//                c.setSense('<');
//                c.addToRHS(vp.vs_dist_cqs.get(sv)[0], 1.0);
//                c.addToRHS(vp.vsCnn_q.get(sv), -M);
//                c.setRHSConstant(M);
//                executor.addConstraint(c);
//                c = new GurobiConstraint();
//                c.setName("sv.3_leq:XY");
//                c.addToLHS(vp.vs_corDist_cqs[1], 1.0);
//                c.setSense('<');
//                c.addToRHS(vp.vs_dist_cqs.get(sv)[1], 1.0);
//                c.addToRHS(vp.vsCnn_q.get(sv), -M);
//                c.setRHSConstant(M);
//                executor.addConstraint(c);




                /*
                d_is_j without Detour
                VVVV
                 */
                c = new GurobiConstraint();
                c.setName("vs_pathLength_min");
                c.addToLHS(vp.vs_dist_cqs.get(sv)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[2], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("vs_pathLength_xy");
                c.addToLHS(vp.vs_dist_cqs.get(sv)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[3], 1.0);
                executor.addConstraint(c);
                auxiliaryDistCons_VarBase(vp.x, vp.y, sv, "dvs_woDetour:v" + virtualPointVars.indexOf(vp), vp.aux_vsDist_iqs.get(sv), vp.auxQ_vsDist.get(sv));
                /*
                AAAA
                d_i_sj
                 */


                buildCons_vpToBaseDetourLength(obstacles, virtualPointVars, vp, sv, vp.vs_dist_cqs.get(sv), vp.vs_dOut_cqs.get(sv), vp.vs_dIn_cqs.get(sv), vp.vs_detour_q.get(sv), vp.vs_relObstacles_qs.get(sv), vp.vs_relObstaclesD_qs.get(sv), vp.vs_relObstacles_q.get(sv), vp.vs_inOutCnn_qs.get(sv), vp.vs_corner_qs.get(sv), vp.vs_outCorner_qs.get(sv), vp.aux_vs_dOutCorner_iqs.get(sv), vp.auxQ_vs_dOutCorner.get(sv), vp.vs_inCorner_qs.get(sv), vp.vs_omOnCnn_q.get(sv), vp.vs_omOnCorner_qs.get(sv), vp.vs_dOmOn_cqs.get(sv), vp.auxQuad_vs_omOnCorner_qs.get(sv), vp.auxQuad_vs_inOutCorner_qs.get(sv));


            }





        }


    }

    private void buildCons_vpToBaseDetourLength(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, VirtualPointVar vp, PseudoBase base, GurobiVariable[] dist_cqs, GurobiVariable[] dOut_cqs, GurobiVariable[] dIn_cqs, GurobiVariable detour_q, Map<Obstacle, GurobiVariable[]> relObstacles_qs, Map<Obstacle, GurobiVariable[]> relObstaclesD_qs, Map<Obstacle, GurobiVariable> relObstacles_q, Map<Obstacle, GurobiVariable[]> inOutCnn_qs, Map<Obstacle, GurobiVariable[]> corner_qs, Map<Obstacle, GurobiVariable[]> outCorner_qs, Map<Obstacle, Map<PseudoBase, GurobiVariable[]>> aux_dOutCorner_iqs, Map<Obstacle, Map<PseudoBase, GurobiVariable>> auxQ_dOutCorner, Map<Obstacle, GurobiVariable[]> inCorner_qs, Map<Obstacle, Map<Obstacle, GurobiVariable>> omOnCnn_q, Map<Obstacle, Map<Obstacle,GurobiVariable[]>> omOnCorner_qs, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> dOmOn_cqs, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> auxQuad_omOn, Map<Obstacle, GurobiVariable[]> auxQuad_inOut) throws GRBException {
        GurobiConstraint c;
        String nickname = "v" + virtualPointVars.indexOf(vp) + "-" + base.getName();
        /*
        d_i_sj regarding Detour
         */
        GurobiConstraint c_vBase_pathLengthMin = new GurobiConstraint();
        c_vBase_pathLengthMin.setName(nickname + "_pathLength_detour_min");
        c_vBase_pathLengthMin.addToLHS(dist_cqs[0], 1.0);
        c_vBase_pathLengthMin.setSense('>');
        c_vBase_pathLengthMin.addToRHS(dOut_cqs[0], 1.0);
        c_vBase_pathLengthMin.addToRHS(dIn_cqs[0], 1.0);
        c_vBase_pathLengthMin.addToRHS(detour_q, M);
        c_vBase_pathLengthMin.setRHSConstant(-M);
        executor.addConstraint(c_vBase_pathLengthMin);
        GurobiConstraint c_vBase_pathLengthXY = new GurobiConstraint();
        c_vBase_pathLengthXY.setName(nickname + "_pathLength_detour_xy");
        c_vBase_pathLengthXY.addToLHS(dist_cqs[1], 1.0);
        c_vBase_pathLengthXY.setSense('>');
        c_vBase_pathLengthXY.addToRHS(dOut_cqs[1], 1.0);
        c_vBase_pathLengthXY.addToRHS(dIn_cqs[1], 1.0);
        c_vBase_pathLengthXY.addToRHS(detour_q, M);
        c_vBase_pathLengthXY.setRHSConstant(-M);
        executor.addConstraint(c_vBase_pathLengthXY);

        /*
        vp->Base: detour_triggering_aux.2: if one obstacle is relevant => detour trigger
        */
        GurobiConstraint c_vBase_detour_geq = new GurobiConstraint();
        c_vBase_detour_geq.setName(nickname + "_detourTriggerAux2_geq");
        c_vBase_detour_geq.addToLHS(detour_q, obstacles.size());
        c_vBase_detour_geq.setSense('>');
        executor.addConstraint(c_vBase_detour_geq);
        GurobiConstraint c_vBase_detour_leq = new GurobiConstraint();
        c_vBase_detour_leq.setName(nickname + "_detourTriggerAux2_leq");
        c_vBase_detour_leq.addToLHS(detour_q, 1.0);
        c_vBase_detour_leq.setSense('<');
        executor.addConstraint(c_vBase_detour_leq);
        //vp->slave: cnnRules.5: if detour => startPoint should connect one of the relevant obstacle
        GurobiConstraint c_vBase_svpToObstacle = new GurobiConstraint();
        c_vBase_svpToObstacle.setName(nickname + "_cnnRules.5");
        c_vBase_svpToObstacle.setSense('=');
        c_vBase_svpToObstacle.addToRHS(detour_q, 1.0);
        executor.addConstraint(c_vBase_svpToObstacle);
        //vp->slave: cnnRules.6: if detour => one of the relevant obstacle should connect to endPoint
        GurobiConstraint c_vBase_obstacleToEp = new GurobiConstraint();
        c_vBase_obstacleToEp.setName(nickname + "_cnnRules.6");
        c_vBase_obstacleToEp.setSense('=');
        c_vBase_obstacleToEp.addToRHS(detour_q, 1.0);
        executor.addConstraint(c_vBase_obstacleToEp);


        for (Obstacle o : obstacles) {

            c_vBase_detour_geq.addToRHS(relObstacles_q.get(o), 1.0);
            c_vBase_detour_leq.addToRHS(relObstacles_q.get(o), 1.0);

            //vBase:cnnRules.5
            //c_vBase_svpToObstacle.addToLHS(inOutCnn_qs.get(o)[0], relObstacles_q.get(o), 1.0);
            c = new GurobiConstraint();
            c.addToLHS(auxQuad_inOut.get(o)[0], 1.0);
            c.setSense('<');
            c.addToRHS(inOutCnn_qs.get(o)[0], 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(auxQuad_inOut.get(o)[0], 1.0);
            c.setSense('<');
            c.addToRHS(relObstacles_q.get(o), 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(auxQuad_inOut.get(o)[0], 1.0);
            c.setSense('>');
            c.addToRHS(inOutCnn_qs.get(o)[0], 1.0);
            c.addToRHS(relObstacles_q.get(o), 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);
            c_vBase_svpToObstacle.addToLHS(auxQuad_inOut.get(o)[0], 1.0);
            //vBase:cnnRules.6
            //c_vBase_obstacleToEp.addToLHS(inOutCnn_qs.get(o)[1], relObstacles_q.get(o), 1.0);
            c = new GurobiConstraint();
            c.addToLHS(auxQuad_inOut.get(o)[1], 1.0);
            c.setSense('<');
            c.addToRHS(inOutCnn_qs.get(o)[1], 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(auxQuad_inOut.get(o)[1], 1.0);
            c.setSense('<');
            c.addToRHS(relObstacles_q.get(o), 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.addToLHS(auxQuad_inOut.get(o)[1], 1.0);
            c.setSense('>');
            c.addToRHS(inOutCnn_qs.get(o)[1], 1.0);
            c.addToRHS(relObstacles_q.get(o), 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);
            c_vBase_obstacleToEp.addToLHS(auxQuad_inOut.get(o)[1], 1.0);


            buildCons_virtualAndBaseOppositeRelation(base, vp, o, relObstacles_qs, relObstaclesD_qs, nickname);


            //Opposite Relation Recognition: vBase_aux.1
            c = new GurobiConstraint();
            c.setName(nickname + "_aux.1");
            c.addToLHS(relObstacles_qs.get(o)[0], 1.0);
            c.addToLHS(relObstacles_qs.get(o)[1], 1.0);
            c.addToLHS(relObstacles_qs.get(o)[2], 1.0);
            c.addToLHS(relObstacles_qs.get(o)[3], 1.0);
            c.addToLHS(relObstaclesD_qs.get(o)[0], 1.0);
            c.addToLHS(relObstaclesD_qs.get(o)[1], 1.0);
            c.addToLHS(relObstaclesD_qs.get(o)[2], 1.0);
            c.addToLHS(relObstaclesD_qs.get(o)[3], 1.0);
            c.setSense('<');
            c.addToRHS(relObstacles_q.get(o), 8.0);
            executor.addConstraint(c);


            /*
            vp->Base: Connection Rules:
             */
            //vBase_corner
            c = new GurobiConstraint();
            c.setName(nickname + "_corner");
            c.addToLHS(corner_qs.get(o)[0], 1.0);
            c.addToLHS(corner_qs.get(o)[1], 1.0);
            c.addToLHS(corner_qs.get(o)[2], 1.0);
            c.addToLHS(corner_qs.get(o)[3], 1.0);
            c.setSense('=');
            c.addToRHS(relObstacles_q.get(o), 1.0);
            executor.addConstraint(c);

            //vp->Base: outCorner: vi -> o
            buildCons_inOutCornerSelection(inOutCnn_qs.get(o)[0], outCorner_qs.get(o), vp.rel_qs.get(o), vp.relD_qs.get(o), nickname + "->o.Corner");
            //vp->Base: inCorner: o -> pj
            int[] rel_q = Arrays.copyOfRange(base.getPseudo_oRel_qs().get(o), 0, 4);
            int[] relD_q = Arrays.copyOfRange(base.getPseudo_oRel_qs().get(o), base.getPseudo_oRel_qs().get(o).length - 4, base.getPseudo_oRel_qs().get(o).length);
            buildCons_inOutCornerSelection(inOutCnn_qs.get(o)[1], inCorner_qs.get(o), rel_q, relD_q, nickname + "o.Corner->");

            //vp->Base:cnnRules.1: q_om->om = 0
            c = new GurobiConstraint();
            c.setName(nickname + "_cnnRules.1");
            c.addToLHS(omOnCnn_q.get(o).get(o), 1.0);
            c.setSense('=');
            c.setRHSConstant(0.0);
            executor.addConstraint(c);

            //vp->Base: cnnRules.3: om relevant => connect to one relevant on or endPoint
            GurobiConstraint c_vBaseObstacleOutCnn = new GurobiConstraint();
            c_vBaseObstacleOutCnn.setName(nickname + "_cnnRules.3");
            c_vBaseObstacleOutCnn.addToLHS(relObstacles_q.get(o), 1.0);
            c_vBaseObstacleOutCnn.setSense('=');
            c_vBaseObstacleOutCnn.addToRHS(inOutCnn_qs.get(o)[1], 1.0);
            executor.addConstraint(c_vBaseObstacleOutCnn);
            //vp->slave: cnnRules.4: om relevant => be connected by one relevant on or startPoint
            GurobiConstraint c_vBaseInObstacleCnn = new GurobiConstraint();
            c_vBaseInObstacleCnn.setName(nickname + "_cnnRules.4");
            c_vBaseInObstacleCnn.addToLHS(relObstacles_q.get(o), 1.0);
            c_vBaseInObstacleCnn.setSense('=');
            c_vBaseInObstacleCnn.addToRHS(inOutCnn_qs.get(o)[0], 1.0);
            executor.addConstraint(c_vBaseInObstacleCnn);

            //vs: Om <-> On
            for (Obstacle on : obstacles) {
                if (!o.getName().equals(on.getName())) {

                    //vp->Base: cornerCnn: q_om->on = sum q_om.corner->on.corner
                    c = new GurobiConstraint();
                    c.setName(nickname + "_cornerCnn");
                    c.addToLHS(omOnCnn_q.get(o).get(on), 1.0);
                    c.setSense('=');
                    for (int i = 0; i < omOnCorner_qs.get(o).get(on).length; ++i){
                        c.addToRHS(omOnCorner_qs.get(o).get(on)[i], 1.0);
                    }
                    executor.addConstraint(c);

                    //vp->Base: cnnRules.2: q_om->on + q_on->om <= 1
                    c = new GurobiConstraint();
                    c.setName(nickname + "_cnnRules.2");
                    c.addToLHS(omOnCnn_q.get(o).get(on), 1.0);
                    c.addToLHS(omOnCnn_q.get(on).get(o), 1.0);
                    c.setSense('<');
                    c.setRHSConstant(1.0);
                    executor.addConstraint(c);

                    //vBase_cnnRules.3
                    //c_vBaseObstacleOutCnn.addToRHS(omOnCnn_q.get(o).get(on), relObstacles_q.get(on), 1.0);
                    //vBase_cnnRules.3_lin
                    c = new GurobiConstraint();
                    c.addToLHS(auxQuad_omOn.get(o).get(on)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(omOnCnn_q.get(o).get(on), 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(auxQuad_omOn.get(o).get(on)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(relObstacles_q.get(on), 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(auxQuad_omOn.get(o).get(on)[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(omOnCnn_q.get(o).get(on), 1.0);
                    c.addToRHS(relObstacles_q.get(on), 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);
                    c_vBaseObstacleOutCnn.addToRHS(auxQuad_omOn.get(o).get(on)[0], 1.0);

                    //vBase_cnnRules.4
                    //c_vBaseInObstacleCnn.addToRHS(omOnCnn_q.get(on).get(o), relObstacles_q.get(o), 1.0);
                    //vBase_cnnRules.4_lin
                    c = new GurobiConstraint();
                    c.addToLHS(auxQuad_omOn.get(on).get(o)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(omOnCnn_q.get(on).get(o), 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(auxQuad_omOn.get(on).get(o)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(relObstacles_q.get(o), 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(auxQuad_omOn.get(on).get(o)[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(omOnCnn_q.get(on).get(o), 1.0);
                    c.addToRHS(relObstacles_q.get(o), 1.0);
                    c.setRHSConstant(-1.0);
                    executor.addConstraint(c);
                    c_vBaseInObstacleCnn.addToRHS(auxQuad_omOn.get(on).get(o)[1], 1.0);



                    for (int cnt = 0; cnt < omOnCorner_qs.get(o).get(on).length; ++cnt){
                        //pathLength.2:Min
                        c = new GurobiConstraint();
                        c.setName(nickname + "_pathLength.2:Min");
                        c.addToLHS(dOmOn_cqs.get(o).get(on)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(omOnCorner_qs.get(o).get(on)[cnt], M);
                        c.setRHSConstant(o.getOo_distMin().get(on)[cnt] - M);
                        executor.addConstraint(c);
                        //pathLength.2:XY
                        c = new GurobiConstraint();
                        c.setName(nickname + "_pathLength.2:XY");
                        c.addToLHS(dOmOn_cqs.get(o).get(on)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(omOnCorner_qs.get(o).get(on)[cnt], M);
                        c.setRHSConstant(o.getOo_distXY().get(on)[cnt] - M);
                        executor.addConstraint(c);

                    }


                    //todo: vs_d_m->n:MIN<=M
                    c = new GurobiConstraint();
                    c.setName(nickname + "_d_m->n:MIN<=M");
                    c.addToLHS(dOmOn_cqs.get(o).get(on)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(omOnCnn_q.get(o).get(on), M);
                    executor.addConstraint(c);

                    c_vBase_pathLengthMin.addToRHS(dOmOn_cqs.get(o).get(on)[0], 1.0);

                    //todo: vs_d_m->n:XY<=M
                    c = new GurobiConstraint();
                    c.setName(nickname + "_d_m->n:XY<=M");
                    c.addToLHS(dOmOn_cqs.get(o).get(on)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(omOnCnn_q.get(o).get(on), M);
                    executor.addConstraint(c);

                    c_vBase_pathLengthXY.addToRHS(dOmOn_cqs.get(o).get(on)[1], 1.0);


                }
            }


            /*
            dvi-> and d->pj: vBase_pathLength.1, vBase_pathLength.3
             */
            for (int cnt = 0; cnt < o.getBaseArray().size(); ++cnt){
                PseudoBase corner = o.getBaseArray().get(cnt);
                //vp->Base: pathLength.1:Min
                c = new GurobiConstraint();
                c.setName(nickname + "_pathLength.1:Min");
                c.addToLHS(dOut_cqs[0], 1.0);
                c.setSense('>');
                c.addToRHS(aux_dOutCorner_iqs.get(o).get(corner)[2], 1.0);
                c.addToRHS(outCorner_qs.get(o)[cnt], M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                //vp->Base: pathLength.1:XY
                c = new GurobiConstraint();
                c.setName(nickname + "_pathLength.1:XY");
                c.addToLHS(dOut_cqs[1], 1.0);
                c.setSense('>');
                c.addToRHS(aux_dOutCorner_iqs.get(o).get(corner)[3], 1.0);
                c.addToRHS(outCorner_qs.get(o)[cnt], M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);

                auxiliaryDistCons_VarBase(vp.x, vp.y, corner, nickname + "dvB_i->", aux_dOutCorner_iqs.get(o).get(corner), auxQ_dOutCorner.get(o).get(corner));

                //vp->slave: pathLength.3:Min
                c = new GurobiConstraint();
                c.setName(nickname + "_pathLength.3:Min");
                c.addToLHS(dIn_cqs[0], 1.0);
                c.setSense('>');
                c.addToRHS(inCorner_qs.get(o)[cnt], M);
                c.setRHSConstant(o.getoBase_distMin().get(base).get(corner) - M);
                executor.addConstraint(c);
                //vp->slave: pathLength.3:XY
                c = new GurobiConstraint();
                c.setName(nickname + "_pathLength.3:XY");
                c.addToLHS(dIn_cqs[1], 1.0);
                c.setSense('>');
                c.addToRHS(inCorner_qs.get(o)[cnt], M);
                c.setRHSConstant(o.getoBase_distXY().get(base).get(corner) - M);
                executor.addConstraint(c);





            }


        }
    }

    private void buildCons_Master(ArrayList<Obstacle> obstacles, PseudoBase master, GurobiConstraint c_busMin, GurobiConstraint c_busDiff, VirtualPointVar vp, ArrayList<VirtualPointVar> virtualPointVars) throws GRBException {
        GurobiConstraint c;





        //VM_dist_wo_Detour
        c = new GurobiConstraint();
        c.setName("vm_pathLength_min");
        c.addToLHS(vp.vm_dist_cqs[0], 1.0);
        c.setSense('>');
        c.addToRHS(vp.aux_vmDist_iqs[2], 1.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("vm_pathLength_xy");
        c.addToLHS(vp.vm_dist_cqs[1], 1.0);
        c.setSense('>');
        c.addToRHS(vp.aux_vmDist_iqs[3], 1.0);
        executor.addConstraint(c);
        auxiliaryDistCons_VarBase(vp.x, vp.y, master, "vm_pathLength_Aux_", vp.aux_vmDist_iqs, vp.auxQ_vmDist);


        c_busMin.addToRHS(vp.vm_dist_cqs[0], 1.0);
        c_busDiff.addToRHS(vp.vm_dist_cqs[1], 1.0);

        buildCons_vpToBaseDetourLength(obstacles, virtualPointVars, vp, master, vp.vm_dist_cqs, vp.vm_dOut_cqs, vp.vm_dIn_cqs, vp.vm_detour_q, vp.vm_relObstacles_qs, vp.vm_relObstaclesD_qs, vp.vm_relObstacles_q, vp.vm_inOutCnn_qs, vp.vm_corner_qs, vp.vm_outCorner_qs, vp.aux_vm_dOutCorner_iqs, vp.auxQ_vm_dOutCorner, vp.vm_inCorner_qs, vp.vm_omOnCnn_q, vp.vm_omOnCorner_qs, vp.vm_dOmOn_cqs, vp.auxQuad_vm_omOnCorner_qs, vp.auxQuad_vm_inOutCorner_qs);

    }


    /**
     *
     * @param inOutCnn_q indicate which obstacle is connected for vi-> or ->p_j
     * @param corner_qs obstacle's [0: o.ll, 1: o.ul, 2: o.ur, 3: o.lr] is connected
     * @param rel_qs which opposite relation of vi and pj regarding o
     * @param relD_qs which direct opposite relation of vi and pj regarding o
     */
    private void buildCons_inOutCornerSelection(GurobiVariable inOutCnn_q, GurobiVariable[] corner_qs, GurobiVariable[] rel_qs, GurobiVariable[] relD_qs, String nickname) {
        GurobiConstraint c;
        c = new GurobiConstraint();
        c.setName(nickname + "_L_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(relD_qs[0], 7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_L_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(relD_qs[0], -7.0);
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_R_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.addToRHS(relD_qs[1], 7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_R_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.addToRHS(relD_qs[1], -7.0);
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_T_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(relD_qs[2], 7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_T_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(relD_qs[2], -7.0);
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_B_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.addToRHS(relD_qs[3], 7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_B_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.addToRHS(relD_qs[3], -7.0);
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_tLbR_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(rel_qs[0], -7.0);
        c.addToRHS(rel_qs[3], -7.0);
        c.setRHSConstant(7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_tLbR_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(rel_qs[0], 7.0);
        c.addToRHS(rel_qs[3], 7.0);
        c.setRHSConstant(-7.0);
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_tRbL_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.addToRHS(rel_qs[1], -7.0);
        c.addToRHS(rel_qs[2], -7.0);
        c.setRHSConstant(7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_tRbL_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.addToRHS(rel_qs[1], 7.0);
        c.addToRHS(rel_qs[2], 7.0);
        c.setRHSConstant(-7.0);
        executor.addConstraint(c);
    }

    /**
     *
     * @param inOutCnn_q indicate which obstacle is connected for vi-> or ->p_j
     * @param corner_qs obstacle's [0: o.ll, 1: o.ul, 2: o.ur, 3: o.lr] is connected
     * @param rel_qs which opposite relation of vi and pj regarding o
     * @param relD_qs which direct opposite relation of vi and pj regarding o
     */
    private void buildCons_inOutCornerSelection(GurobiVariable inOutCnn_q, GurobiVariable[] corner_qs, int[] rel_qs, int[] relD_qs, String nickname) {
        GurobiConstraint c;
        c = new GurobiConstraint();
        c.setName(nickname + "_L_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[1], 1.0);
        c.setRHSConstant(7 * relD_qs[0]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_L_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[1], 1.0);
        c.setRHSConstant(relD_qs[0] * (-7.0));
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_R_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.setRHSConstant(7.0 * relD_qs[1]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_R_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[2], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.setRHSConstant(relD_qs[1] * (-7.0));
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_T_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.setRHSConstant(relD_qs[2] * 7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_T_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.setRHSConstant(relD_qs[2] * (-7.0));
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_B_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.setRHSConstant(relD_qs[3] * 7.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_B_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.setRHSConstant(relD_qs[3] * (-7.0));
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_tLbR_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.setRHSConstant((1.0 - rel_qs[0] - rel_qs[3]) * (7.0));
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_tLbR_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], 1.0);
        c.addToRHS(corner_qs[2], 1.0);
        c.setRHSConstant((1.0 - rel_qs[0] - rel_qs[3]) * (-7.0));
        executor.addConstraint(c);

        c = new GurobiConstraint();
        c.setName(nickname + "_outCorner_tRbL_leq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.setRHSConstant((1 - rel_qs[1] - rel_qs[2]) * (7.0));
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_outCorner_tRbL_geq");
        c.addToLHS(inOutCnn_q, 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[1], 1.0);
        c.addToRHS(corner_qs[3], 1.0);
        c.setRHSConstant((1 - rel_qs[1] - rel_qs[2]) * (-7.0));
        executor.addConstraint(c);
    }




    private void auxiliaryDistCons_VarBase(GurobiVariable x, GurobiVariable y, PseudoBase base, String nickName, GurobiVariable[] aux_dist_iqs, GurobiVariable auxQ_dist) throws GRBException {
        GurobiConstraint c;
        //aux
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_X");
        c.addToLHS(x, 1.0);
        c.setLHSConstant(-base.getX());
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[4], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[0], aux_dist_iqs[4], nickName + "->" + base.getName() + "_absX");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_Y");
        c.addToLHS(y, 1.0);
        c.setLHSConstant(-base.getY());
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[5], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[1], aux_dist_iqs[5], nickName + "->" + base.getName() + "_absY");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_XY");
        c.addToLHS(aux_dist_iqs[0], 1.0);
        c.addToLHS(aux_dist_iqs[1], -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[6], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[3], aux_dist_iqs[6], nickName + "->" + base.getName() + "_absXY");
        //Min_linear
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[0], 1.0);
        c.addToRHS(auxQ_dist, M);
        c.setRHSConstant(-M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[1], 1.0);
        c.addToRHS(auxQ_dist, -M);
        executor.addConstraint(c);
    }

    private void auxiliaryDistCons_VarVar(GurobiVariable x1, GurobiVariable x2, GurobiVariable y1, GurobiVariable y2, String nickName, GurobiVariable[] aux_dist_iqs, GurobiVariable auxQ_dist) throws GRBException {
        GurobiConstraint c;

        c = new GurobiConstraint();
        c.setName(nickName + "_aux_x");
        c.addToLHS(x1, 1.0);
        c.addToLHS(x2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[4], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[0], aux_dist_iqs[4], nickName + "absX");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_Y");
        c.addToLHS(y1, 1.0);
        c.addToLHS(y2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[5], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[1], aux_dist_iqs[5], nickName + "absY");
        //Min
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[0], 1.0);
        c.addToRHS(auxQ_dist, M);
        c.setRHSConstant(-M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[1], 1.0);
        c.addToRHS(auxQ_dist, -M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_XY");
        c.addToLHS(aux_dist_iqs[0], 1.0);
        c.addToLHS(aux_dist_iqs[1], -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[6], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[3], aux_dist_iqs[6], nickName + "abs_XY");
    }


    private void buildCons_virtualAndBaseOppositeRelation(PseudoBase base, VirtualPointVar vp, Obstacle o, Map<Obstacle, GurobiVariable[]> vpRelObstacle_qs, Map<Obstacle, GurobiVariable[]> vpRelObstacleD_qs, String nickname) {
        GurobiConstraint c;
        //1.ul->lr
        c = new GurobiConstraint();
        c.setName(nickname + "ul->lr.2_o");
        c.addToLHS(vpRelObstacle_qs.get(o)[0], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_bRObstacles()) {
            //1.ul->lr.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "ul->lr.1_o");
            c.addToLHS(vpRelObstacle_qs.get(o)[0], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[3] - 1.0);
            executor.addConstraint(c);

            //1.ul->lr.1_on
            c = new GurobiConstraint();
            c.setName(nickname + "ul->lr.1_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[0], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[3] - 1.0);
            executor.addConstraint(c);
            //1.ul->lr.2_on
            c = new GurobiConstraint();
            c.setName(nickname + "ul->lr.2_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[0], 2.0);
            c.setSense('<');
            c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[3]);
            executor.addConstraint(c);

        }

        //2.lr->ul
        c = new GurobiConstraint();
        c.setName(nickname + "lr->ul.2_o");
        c.addToLHS(vpRelObstacle_qs.get(o)[1], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_tLObstacles()) {
            //2.lr->ul.1
            c = new GurobiConstraint();
            c.setName(nickname + "lr->ul.1_o");
            c.addToLHS(vpRelObstacle_qs.get(o)[1], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[0] - 1.0);
            executor.addConstraint(c);

            //2.lr->ul.1_on
            c = new GurobiConstraint();
            c.setName(nickname + "lr->ul.1_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[1], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[0] - 1.0);
            executor.addConstraint(c);
            //2.lr->ul.2_on
            c = new GurobiConstraint();
            c.setName(nickname + "lr->ul.2_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[1], 2.0);
            c.setSense('<');
            c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[0]);
            executor.addConstraint(c);

        }


        //3:ur->ll
        c = new GurobiConstraint();
        c.setName(nickname + "ur->ll.2_o");
        c.addToLHS(vpRelObstacle_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_bLObstacles()) {
            //3:ur->ll.1
            c = new GurobiConstraint();
            c.setName(nickname + "ur->ll.1_o");
            c.addToLHS(vpRelObstacle_qs.get(o)[2], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[2] - 1.0);
            executor.addConstraint(c);

            //3:ur->ll.1_on
            c = new GurobiConstraint();
            c.setName(nickname + "ur->ll.1_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[2], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[2] - 1.0);
            executor.addConstraint(c);
            //3:ur->ll.2_on
            c = new GurobiConstraint();
            c.setName(nickname + "ur->ll.2_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[2], 2.0);
            c.setSense('<');
            c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[2]);
            executor.addConstraint(c);


        }


        //4:ll->ur
        c = new GurobiConstraint();
        c.setName(nickname + "ll->ur.2_o");
        c.addToLHS(vpRelObstacle_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_tRObstacles()) {
            //4:ll->ur.1
            c = new GurobiConstraint();
            c.setName(nickname + "ll->ur.1_o");
            c.addToLHS(vpRelObstacle_qs.get(o)[3], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[1] - 1.0);
            executor.addConstraint(c);


            //4:ll->ur.1_on
            c = new GurobiConstraint();
            c.setName(nickname + "ll->ur.1_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[3], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[1] - 1.0);
            executor.addConstraint(c);
            //4:ll->ur.2_on
            c = new GurobiConstraint();
            c.setName(nickname + "ll->ur.2_on");
            c.addToLHS(vpRelObstacle_qs.get(on)[3], 2.0);
            c.setSense('<');
            c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[1]);
            executor.addConstraint(c);

        }

        //(1)l->r
        c = new GurobiConstraint();
        c.setName(nickname + "l->r.2_o");
        c.addToLHS(vpRelObstacleD_qs.get(o)[0], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dRObstacles()) {
            //(1)l->r.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "l->r.1_o");
            c.addToLHS(vpRelObstacleD_qs.get(o)[0], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[5] - 1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //(1)l->r.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "l->r.1_on");
                c.addToLHS(vpRelObstacleD_qs.get(on)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[5] - 1.0);
                executor.addConstraint(c);

                //(1)l->r.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "l->r.2_on");
                c.addToLHS(vpRelObstacleD_qs.get(on)[0], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[5]);
                executor.addConstraint(c);

            }


        }


        //(2)r->l
        c = new GurobiConstraint();
        c.setName(nickname + "r->l.2_o");
        c.addToLHS(vpRelObstacleD_qs.get(o)[1], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dLObstacles()) {
            //(2)r->l.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "r->l.1_o");
            c.addToLHS(vpRelObstacleD_qs.get(o)[1], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[4] - 1.0);
            executor.addConstraint(c);
            if (!o.getName().equals(on.getName())) {

                //(2)r->l.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "r->l.1_o");
                c.addToLHS(vpRelObstacleD_qs.get(on)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[4] - 1.0);
                executor.addConstraint(c);

                //(2)r->l.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "r->l.2_on");
                c.addToLHS(vpRelObstacleD_qs.get(on)[1], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[4]);
                executor.addConstraint(c);

            }


        }

        //(3)t->b
        c = new GurobiConstraint();
        c.setName(nickname + "t->b.2_o");
        c.addToLHS(vpRelObstacleD_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dBObstacles()) {
            //(3)t->b.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "t->b.1_o");
            c.addToLHS(vpRelObstacleD_qs.get(o)[2], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[7] - 1.0);
            executor.addConstraint(c);
            if (!o.getName().equals(on.getName())) {

                //(3)t->b.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "t->b.1_on");
                c.addToLHS(vpRelObstacleD_qs.get(on)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[7] - 1.0);
                executor.addConstraint(c);

                //(3)t->b.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "t->b.2_on");
                c.addToLHS(vpRelObstacleD_qs.get(on)[2], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[7]);
                executor.addConstraint(c);

            }


        }
        //(4)b->t
        c = new GurobiConstraint();
        c.setName(nickname + "b->t.2_o");
        c.addToLHS(vpRelObstacleD_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dTObstacles()) {
            //(4)b->t.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "b->t.1_o");
            c.addToLHS(vpRelObstacleD_qs.get(o)[3], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[6] - 1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //(4)b->t.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "b->t.1_on");
                c.addToLHS(vpRelObstacleD_qs.get(on)[3], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[6] - 1.0);
                executor.addConstraint(c);

                //(4)b->t.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "b->t.2_on");
                c.addToLHS(vpRelObstacleD_qs.get(on)[3], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[6]);
                executor.addConstraint(c);

            }


        }
    }

    private void buildCons_nonOverlapAndOppositeRelation(double minDist, VirtualPointVar vp, Obstacle o) {
        GurobiConstraint c;
        //nonOverlapping
        c = new GurobiConstraint();
        c.setName("nonl");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.setRHSConstant(3.0);
        executor.addConstraint(c);
        //nonl.l
        c = new GurobiConstraint();
        c.setName("nonl.l_1");
        c.addToLHS(vp.x, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[0], M);
        c.setRHSConstant(o.getMinX() - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.l_2");
        c.addToLHS(vp.x, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[0], M);
        c.setRHSConstant(o.getMinX() - minDist);
        executor.addConstraint(c);
        //nonl.r
        c = new GurobiConstraint();
        c.setName("nonl.r_1");
        c.addToLHS(vp.x, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[1], -M);
        c.setRHSConstant(o.getMaxX() + M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.r_2");
        c.addToLHS(vp.x, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[1], -M);
        c.setRHSConstant(o.getMaxX() + minDist);
        executor.addConstraint(c);
        //nonl.t
        c = new GurobiConstraint();
        c.setName("nonl.t_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMaxY() + M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.t_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMaxY() + minDist);
        executor.addConstraint(c);
        //nonl.b
        c = new GurobiConstraint();
        c.setName("nonl.b_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[3], M);
        c.setRHSConstant(o.getMinY() - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.b_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[3], M);
        c.setRHSConstant(o.getMinY() - minDist);
        executor.addConstraint(c);


        //diagonal
        //rel.ul
        c = new GurobiConstraint();
        c.setName("rel.ul_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[0], M);
        c.setRHSConstant(o.getMaxY() - o.getMinX() + minDist - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.ul_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[0], M);
        c.setRHSConstant(o.getMaxY() - o.getMinX());
        executor.addConstraint(c);

        //rel.ur
        c = new GurobiConstraint();
        c.setName("rel.ur_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[1], M);
        c.setRHSConstant(o.getMaxY() + o.getMaxX() + minDist - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.ur_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[1], M);
        c.setRHSConstant(o.getMaxY() + o.getMaxX());
        executor.addConstraint(c);

        //rel.lr
        c = new GurobiConstraint();
        c.setName("rel.lr_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMinY() - o.getMaxX());
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.lr_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMinY() - o.getMaxX() - minDist + M);
        executor.addConstraint(c);

        //rel.ll
        c = new GurobiConstraint();
        c.setName("rel.ll_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[3], -M);
        c.setRHSConstant(o.getMinY() + o.getMinX());
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.ll_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[3], -M);
        c.setRHSConstant(o.getMinY() + o.getMinX() - minDist + M);
        executor.addConstraint(c);


        //tL
        c = new GurobiConstraint();
        c.setName("tL.1");
        c.addToLHS(vp.dir_qs.get(o)[1], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[3], 1.0);
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[0], -4.0);
        c.setRHSConstant(4.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("tL.geq");
        c.addToLHS(vp.dir_qs.get(o)[1], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[3], 1.0);
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.setSense('>');
        c.addToRHS(vp.rel_qs.get(o)[0], -1.0);
        c.setRHSConstant(1.0);
        executor.addConstraint(c);


        //tR
        c = new GurobiConstraint();
        c.setName("tR.1");
        c.addToLHS(vp.dir_qs.get(o)[0], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[1], -4.0);
        c.setRHSConstant(4.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("tR.2");
        c.addToLHS(vp.dir_qs.get(o)[0], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.setSense('>');
        c.addToRHS(vp.rel_qs.get(o)[1], -1.0);
        c.setRHSConstant(1.0);
        executor.addConstraint(c);


        //bL
        c = new GurobiConstraint();
        c.setName("bL.1");
        c.addToLHS(vp.dir_qs.get(o)[0], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[2], -4.0);
        c.setRHSConstant(4.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("bL.2");
        c.addToLHS(vp.dir_qs.get(o)[0], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('>');
        c.addToRHS(vp.rel_qs.get(o)[2], -1.0);
        c.setRHSConstant(1.0);
        executor.addConstraint(c);


        //bR
        c = new GurobiConstraint();
        c.setName("bR.1");
        c.addToLHS(vp.dir_qs.get(o)[1], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[3], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[3], -4.0);
        c.setRHSConstant(4.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("bR.2");
        c.addToLHS(vp.dir_qs.get(o)[1], 1.0);
        c.addToLHS(vp.dir_qs.get(o)[3], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('>');
        c.addToRHS(vp.rel_qs.get(o)[3], -1.0);
        c.setRHSConstant(1.0);
        executor.addConstraint(c);

        //Directly Opposite Relations
        //L
        c = new GurobiConstraint();
        c.setName("dL.1");
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('>');
        c.addToRHS(vp.relD_qs.get(o)[0], 3.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("dL.2");
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);

        //R
        c = new GurobiConstraint();
        c.setName("dR.1");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('>');
        c.addToRHS(vp.relD_qs.get(o)[1], 3.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("dR.2");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);

        //T
        c = new GurobiConstraint();
        c.setName("dT.1");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('>');
        c.addToRHS(vp.relD_qs.get(o)[2], 3.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("dT.2");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);

        //B
        c = new GurobiConstraint();
        c.setName("dB.1");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.setSense('>');
        c.addToRHS(vp.relD_qs.get(o)[3], 3.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("dB.2");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
    }


    public void buildVars(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master) {

        GurobiVariable q;

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
//        //lb_x
//        if (lb_x <= 0) {
//            lb_x *= 2;
//        } else {
//            lb_x *= 0.5;
//        }
//        //lb_y
//        if (lb_y <= 0) {
//            lb_y *= 2;
//        } else {
//            lb_y *= 0.5;
//        }
//        //ub_x
//        if (ub_x >= 0) {
//            ub_x *= 1.2;
//        } else {
//            ub_x *= 0.5;
//        }
//        //ub_y
//        if (ub_y >= 0) {
//            ub_y *= 1.2;
//        } else {
//            ub_y *= 0.5;
//        }

        int lb = Math.min(lb_x, lb_y);
        int ub = Math.max(ub_x, ub_y);



        /*
        initialize virtual points
         */
        for (int i = 0; i < slaves.size(); ++i) {
            VirtualPointVar vp = new VirtualPointVar();
            virtualPointVars.add(vp);
            vp.x = new GurobiVariable(GRB.INTEGER, lb_x, ub_x, "x" + i);
//            vp.x = new GurobiVariable(GRB.INTEGER, 0, 0, "x" + i);
            executor.addVariable(vp.x);
            vp.y = new GurobiVariable(GRB.INTEGER, lb_y, ub_y, "y" + i);
//            vp.y = new GurobiVariable(GRB.INTEGER, 0, 0, "y" + i);
            executor.addVariable(vp.y);


            for (Obstacle o : obstacles) {

                /*
                Regarding Master
                 */
                if (i == 0) {

                    /*
                    VM_relObstacles_qs
                    0: ul->lr
                    1: lr->ul
                    2: ur->ll
                    3: ll->ur
                     */
                    vp.vm_relObstacles_qs.put(o, buildBinaryVar(o.getName() + "_vm_relObstacles_qs", 4));

                    /*
                    vm:relObstaclesD_qs
                    0: l->r
                    1: r->l
                    2: t->b
                    3: b->t
                     */
                    vp.vm_relObstaclesD_qs.put(o, buildBinaryVar(o.getName() + "_vm_relObstacles_qs", 4));

                    /*
                    indicate relevant obstacles
                     */
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + o.getName() + "_vm_relObstacles_q");
                    executor.addVariable(q);
                    vp.vm_relObstacles_q.put(o, q);
                    /*
                    VM_corner_qs
                    0: ll
                    1: ur
                    2: ul
                    3: lr
                     */
                    vp.vm_corner_qs.put(o, buildBinaryVar(o.getName() + "_vm_corner_qs_", 4));

                    /*
                    VM_inOutCnn_qs
                    0: vi->
                    1: vj<-
                     */
                    vp.vm_inOutCnn_qs.put(o, buildBinaryVar(o.getName() + "_vm_inOutCnn_qs_", 2));
                    /*
                    vm_outCorner_qs
                    0: vi->o.ll
                    1: vi->o.ul
                    2: vi->o.ur
                    3: vi->o.lr
                     */
                    vp.vm_outCorner_qs.put(o, buildBinaryVar("v0->o.Corner_q_", 4));

                    /*
                    vm_inCorner_qs
                    0: o.ll->vi+1
                    1: o.ul->vi+1
                    2: o.ur->vi+1
                    3: o.lr->vi+1
                     */
                    vp.vm_inCorner_qs.put(o, buildBinaryVar("o.Corner->ms_q_", 4));
                    /*
                    auxQuad_vm_inOutCorner_qsMap
                    0: cnnRules.5
                    1: cnnRules.6
                     */
                    vp.auxQuad_vm_inOutCorner_qs.put(o, buildBinaryVar("auxQuad_vm_inOutCorner_qs_",2));
                    /*
                    vm_omOnCnn_q
                    q_i_vj^m->n
                     */
                    Map<Obstacle, GurobiVariable> vm_omOnCnn_qMap = new HashMap<>();
                    /*
                    vm_dOmOn_cqs
                    0: d_m->n:MIN
                    1: d_m->n:DIFF
                     */
                    Map<Obstacle, GurobiVariable[]> vm_dOmOn_cqsMap = new HashMap<>();
                    /*
                    16 connections
                     */
                    Map<Obstacle, GurobiVariable[]> vm_omOnCorner_qsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> auxQuad_vm_omOnCorner_qsMap = new HashMap<>();
                    for (Obstacle on : obstacles) {
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "_vm_omOnCnn_q");
                        executor.addVariable(q);
                        vm_omOnCnn_qMap.put(on, q);

                        vm_dOmOn_cqsMap.put(on, buildIntVar(0, M, "v0" + ";" + o.getName() + "->" + on.getName() + "_vm_dOmOn_cqs_", 2));

                        vm_omOnCorner_qsMap.put(on, buildBinaryVar("v0" + ";" + o.getName() + "->" + on.getName() + "_vm_omOnCorner_qs_", 16));
                        auxQuad_vm_omOnCorner_qsMap.put(on, buildBinaryVar("v0" + ";" + o.getName() + "->" + on.getName() + "auxQuad_vm_omOnCorner_qsMap_", 2));
                    }
                    vp.vm_omOnCnn_q.put(o, vm_omOnCnn_qMap);
                    vp.vm_dOmOn_cqs.put(o, vm_dOmOn_cqsMap);
                    vp.vm_omOnCorner_qs.put(o, vm_omOnCorner_qsMap);
                    vp.auxQuad_vm_omOnCorner_qs.put(o, auxQuad_vm_omOnCorner_qsMap);

                    //
                    Map<PseudoBase, GurobiVariable[]> aux_vm_dOutCorner_iqsMap = new HashMap<>();
                    Map<PseudoBase, GurobiVariable> auxQ_vm_dOutCornerMap = new HashMap<>();
                    for (PseudoBase corner : o.getBaseArray()){
                        aux_vm_dOutCorner_iqsMap.put(corner, buildAuxIntVar("aux_vm_dOutCorner_" + corner.getName() + "_iqs_"));
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "auxQ_vm_dOutCorner_" + corner.getName());
                        executor.addVariable(q);
                        auxQ_vm_dOutCornerMap.put(corner, q);
                    }
                    vp.aux_vm_dOutCorner_iqs.put(o, aux_vm_dOutCorner_iqsMap);
                    vp.auxQ_vm_dOutCorner.put(o, auxQ_vm_dOutCornerMap);

                }



                /*
                Non-overlapping
                0: nonL
                1: nonR
                2: nonA
                3: nonB
                 */
                vp.non_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_non_qs_", 4));

                /*
                dir_qs
                0: ul
                1: ur
                2: lr
                3: ll
                 */
                vp.dir_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_dir_qs_", 4));

                /*
                rel_qs
                0: o_tL
                1: o_tR
                2: o_bL
                3: o_bR
                 */
                vp.rel_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_rel_qs_", 4));
                /*
                relD_qs
                0: o_L
                1: o_R
                2: o_T
                3: o_B
                 */
                vp.relD_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_relD_qs_", 4));

                /*
                o_vp_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: relative obstacle: aux.2
                 */
                vp.relObstacles_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_relObstacles_qs_", 4));

                /*
                relObstaclesD_qs
                0: l->r
                1: r->l
                2: t->b
                3: b->t
                 */
                vp.relObstaclesD_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_relObstacles_qs_", 4));

                /*
                indicate the relevance of each obstacle
                 */
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + o.getName() + "_relObstacles_q");
                executor.addVariable(q);
                vp.relObstacles_q.put(o, q);

                /*
                corner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                vp.corner_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_corner_qs_", 4));

                /*
                inOutCnn_qs
                0: ->
                1: <-
                */
                vp.inOutCnn_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_inOutCnn_qs_", 2));
                /*
                outCorner_qs
                0: vi->o.ll
                1: vi->o.ul
                2: vi->o.ur
                3: vi->o.lr
                 */
                vp.outCorner_qs.put(o, buildBinaryVar("v_" + i + "->" + o.getName() + "_outCorner_qs_", 4));

                /*
                inCorner_qs
                0: o.ll->vi+1
                1: o.ul->vi+1
                2: o.ur->vi+1
                3: o.lr->vi+1
                 */
                vp.inCorner_qs.put(o, buildBinaryVar(o.getName() + "->v" + (i + 1) + "_inCorner_qs_", 4));

                vp.auxQuad_inOutCorners_qs.put(o, buildBinaryVar("v_" + i + "->" + o.getName() + "->v" + (i + 1) + "_AuxQuad_inOutCorner_qs_", 2));
                /*
                omOnCnn_q
                0: q_ij^m->n
                 */
                Map<Obstacle, GurobiVariable> omOnCnn_qMap = new HashMap<>();
                /*
                omOnCorner_qs
                0: ll->ll
                1: ll->ul
                2: ll->ur
                3: ll->lr

                4: ul->ll
                5: ul->ul
                6: ul->ur
                7: ul->lr

                8: ur->ll
                9: ur->ul
                10: ur->ur
                11: ur->lr

                12: lr->ll
                13: lr->ul
                14: lr->ur
                15: lr->lr
                 */
                Map<Obstacle, GurobiVariable[]> omOnCorner_qsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> auxQuad_omOnCorner_qsMap = new HashMap<>();
                /*
                dOmOn_cqsMap
                0: d_m->n
                 */
                Map<Obstacle, GurobiVariable[]> dOmOn_cqsMap = new HashMap<>();

                for (Obstacle on : obstacles) {

                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + "-" + on.getName() + "_omOnCnn_q_");
                    executor.addVariable(q);
                    omOnCnn_qMap.put(on, q);


                    omOnCorner_qsMap.put(on, buildBinaryVar("v_" + i + ";" + o.getName() + "->" + on.getName() + "_omOnCorner_qs_", 16));
                    auxQuad_omOnCorner_qsMap.put(on, buildBinaryVar("v_" + i + ";" + o.getName() + "->" + on.getName() + "_AuxQuad_omOnCorner_qs_", 2));
                    dOmOn_cqsMap.put(on, buildIntVar(0, M, "v_" + i + ";" + o.getName() + "->" + on.getName() + "_dOmOn_cqs_", 2));



                }
                vp.omOnCnn_q.put(o, omOnCnn_qMap);
                vp.omOnCorner_qs.put(o, omOnCorner_qsMap);
                vp.auxQuad_omOnCorner_qs.put(o, auxQuad_omOnCorner_qsMap);

                vp.dOmOn_cqs.put(o, dOmOn_cqsMap);




                /*
                Auxiliary absolute values: aux_dOutCorner_iqs/aux_dInCorner_iqs
                0: |vi.x - x_m|
                1: |vi.y - y_m|
                2: min(|vi.x - x_m|, |vi.y - y_m|)
                3: ||vi.x - x_m| - |vi.y - y_m||
                4: vi.x - x_m
                5: vi.y - y_m
                6: |vi.x - x_m| - |vi.y - y_m|
                 */
                Map<PseudoBase, GurobiVariable[]> aux_dOutCorner_iqsMap = new HashMap<>();
                Map<PseudoBase, GurobiVariable> auxQ_dOutCorner = new HashMap<>();
                Map<PseudoBase, GurobiVariable[]> aux_dInCorner_iqsMap = new HashMap<>();
                Map<PseudoBase, GurobiVariable> auxQ_dInCorner = new HashMap<>();
                for (PseudoBase corner : o.getBaseArray()){
                    aux_dOutCorner_iqsMap.put(corner, buildAuxIntVar("v_" + i + ";" + corner.getName() + "_aux_dOutCorner_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i +";" + corner.getName() + "_auxQ_dOutCorner");
                    executor.addVariable(q);
                    auxQ_dOutCorner.put(corner, q);

                    aux_dInCorner_iqsMap.put(corner, buildAuxIntVar("v_" + i + ";" + corner.getName() + "_aux_dInCorner_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i +";" + corner.getName() + "_auxQ_dInCorner");
                    executor.addVariable(q);
                    auxQ_dInCorner.put(corner, q);
                }
                vp.aux_dOutCorner_iqs.put(o, aux_dOutCorner_iqsMap);
                vp.auxQ_dOutCorner.put(o, auxQ_dOutCorner);
                vp.aux_dInCorner_iqs.put(o, aux_dInCorner_iqsMap);
                vp.auxQ_dInCorner.put(o, auxQ_dInCorner);


            }

            /*
            Only for 1st virtualPoint, no rel. to obstacles
             */
            if (i == 0) {

                vp.vm_dist_cqs = buildIntVar(0, M, "_vm_dist_cqs_", 2);

                /*
                VM_detour_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: q_ij^d: detour trigger: aux.2/cnn.4
                 */
                vp.vm_detour_q = new GurobiVariable(GRB.BINARY, 0, 1, "vm_detour_qs");
                executor.addVariable(vp.vm_detour_q);

                /*
                Auxiliary absolute values: aux_vmDist_iqs
                0: |vi.x - ms.x|
                1: |vi.y - ms.y|
                2: min(|vi.x - ms.x|, |vi.y - ms.y|)
                3: ||vi.x - ms.x| - |vi.y - ms.y||
                4: vi.x - ms.x
                5: vi.y - ms.y
                6: |vi.x - ms.x| - |vi.y - ms.y|
                 */
                vp.aux_vmDist_iqs = buildAuxIntVar("v" + i + "_aux_vmDist_iqs_");
                vp.auxQ_vmDist = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_auxQ_vmDist");
                executor.addVariable(vp.auxQ_vmDist);

                /*
                vm_dOut_cqs
                0: v0->: min
                1: v0->: xy
                 */
                vp.vm_dOut_cqs = buildIntVar(0, M, "dv0->_", 2);
                /*
                vm_dIn_cqs
                0: ->ms: Min
                1: ->ms: Diff
                 */
                vp.vm_dIn_cqs = buildIntVar(0, M, "d->ms_", 2);

            }
            /*
            detour_qsdetour trigger:
             */
            vp.detour_q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_detour_q");
            executor.addVariable(vp.detour_q);


            /*
            dist_cqs: vv_dist
            0: dvv:MIN
            1: dvv:DIFF
             */
            vp.dist_cqs = buildIntVar(0, M, "v" + i + "dist_cqs_", 2);
            /*
            Auxiliary absolute values: aux_dist_iqs
            0: |vi.x-vj.x|
            1: |vi.y-j.y|
            2: min(|vi.x-vj.x|,|vi.y-j.y|)
            3: ||vi.x-vj.x|-|vi.y-j.y||
            4: vi.x-vj.x
            5: vi.y-j.y
            6: |vi.x-vj.x|-|vi.y-j.y|
             */
            vp.aux_dist_iqs = buildAuxIntVar("v" + i + "_aux_dist_iqs_");
            vp.auxQ_dist = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_auxQ_dist");
            executor.addVariable(vp.auxQ_dist);

            /*
            d_vCorS: dist between virtualPoint and corresponding slave
            0: d_vCorS:MIN
            1: d_vCors:DIFF
             */
            vp.vs_corDist_cqs = buildIntVar(0, M, "v" + i + "_corDist_cqs_", 2);

            /*
            dOut_cqs
            0: dvi->:MIN
            1: dvi->:DIFF
             */
            vp.dOut_cqs = buildIntVar(0, M, "v" + i + "_dOut_cqs_", 2);
            /*
            dIn_cqs
            0: d->pj:MIN
            1: d->pj:DIFF
             */
            vp.dIn_cqs = buildIntVar(0, M, "v" + i + "_dIn_cqs_", 2);





            /*
            Regarding Slaves
             */
            for (PseudoBase sv : slaves) {
                //vsCnn_q
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_vsCnn_q_" + sv.getName());
                executor.addVariable(q);
                vp.vsCnn_q.put(sv, q);


                /*
                vs_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                */
                Map<Obstacle, GurobiVariable[]> vs_relObstacles_qsMap = new HashMap<>();

                /*
                vs_relObstaclesD_qs
                0: l->r
                1: r->l
                2: t->b
                3: b->t
                 */
                Map<Obstacle, GurobiVariable[]> vs_relObstaclesD_qsMap = new HashMap<>();
                /*
                indicate relevant Obstacles_VS
                 */
                Map<Obstacle, GurobiVariable> vs_relObstaclesD_qMap = new HashMap<>();


                /*
                vs_corner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                Map<Obstacle, GurobiVariable[]> vs_corner_qsMap = new HashMap<>();
                /*
                vs_omOnCnn_q
                q_i_sj^m->n
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable>> vs_omOnCnn_qMap = new HashMap<>();
                /*
                vs_omOnCorner_qs
                0: ll->ll
                1: ll->ul
                2: ll->ur
                3: ll->lr

                4: ul->ll
                5: ul->ul
                6: ul->ur
                7: ul->lr

                8: ur->ll
                9: ur->ul
                10: ur->ur
                11: ur->lr

                12: lr->ll
                13: lr->ul
                14: lr->ur
                15: lr->lr
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vs_omOnCorner_qsMap = new HashMap<>();
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> auxQuad_vs_omOnCorner_qsMap = new HashMap<>();

                /*
                vs_inOutCnn_qs
                0: vi_s ->
                1: sj<-
                 */
                Map<Obstacle, GurobiVariable[]> vs_inOutCnn_qsMap = new HashMap<>();
                /*
                vs_outCorner_qs
                0: vi->o.ll
                1: vi->o.ul
                2: vi->o.ur
                3: vi->o.lr
                 */
                Map<Obstacle, GurobiVariable[]> vs_outCorner_qsMap = new HashMap<>();
                /*
                vs_inCorner_qs
                0: o.ll->sj
                1: o.ul->sj
                2: o.ur->sj
                3: o.lr->sj
                 */
                Map<Obstacle, GurobiVariable[]> vs_inCorner_qsMap = new HashMap<>();

                Map<Obstacle, GurobiVariable[]> auxQuad_vs_inOutCorner_qsMap = new HashMap<>();

                /*
                vs_dOmOn_cqs
                0: vs_d_m->n:MIN
                1: vs_d_m->n:DIFF
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vs_dOmOn_cqsMap = new HashMap<>();


                /*
                Auxiliary absolute values: aux_vsdOut_iqs
                0: |vi.x - x_m|
                1: |vi.y - y_m|
                2: min(|vi.x - x_m|, |vi.y - y_m|)
                3: ||vi.x - x_m| - |vi.y - y_m||
                4: vi.x - x_m
                5: vi.y - y_m
                6: |vi.x - x_m| - |vi.y - y_m|
                 */
                Map<Obstacle, Map<PseudoBase, GurobiVariable[]>> aux_vsdOut_iqsMap = new HashMap<>();
                Map<Obstacle, Map<PseudoBase, GurobiVariable>> auxQ_vsdOutMap = new HashMap<>();


                for (Obstacle o : obstacles) {
                    vs_relObstacles_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_relObstacles_qs_", 4));
                    vs_relObstaclesD_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_relObstaclesD_qs_", 4));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_relObstaclesD_q");
                    executor.addVariable(q);
                    vs_relObstaclesD_qMap.put(o, q);

                    vs_corner_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_corner_qs_", 4));


                    Map<Obstacle, GurobiVariable> vs_onCnn_qMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> vs_onCorner_qsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> auxQuad_vs_onCorner_qsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> vs_dOn_cqsMap = new HashMap<>();

                    for (Obstacle on : obstacles) {
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + on.getName() + "_vs_omOnCnn_q");
                        executor.addVariable(q);
                        vs_onCnn_qMap.put(on, q);
                        vs_onCorner_qsMap.put(on, buildBinaryVar("v" + i +"-" + o.getName() + ";" + sv.getName() + on.getName() + "_vs_omOnCorner_qs_", 16));
                        auxQuad_vs_onCorner_qsMap.put(on, buildBinaryVar("v" + i +"-" + o.getName() + ";" + sv.getName() + on.getName() + "_AuxQuad_vs_omOnCorner_qs_", 2));
                        vs_dOn_cqsMap.put(on, buildIntVar(0, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + ";" + on.getName() + "_vs_dOmOn_cqs_", 2));

                    }


                    vs_omOnCnn_qMap.put(o, vs_onCnn_qMap);
                    vs_omOnCorner_qsMap.put(o, vs_onCorner_qsMap);
                    auxQuad_vs_omOnCorner_qsMap.put(o, auxQuad_vs_onCorner_qsMap);

                    vs_inOutCnn_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_inCnn_q_", 2));
                    vs_outCorner_qsMap.put(o, buildBinaryVar("v" + i + "-" + o.getName() + "-" + sv.getName() + "_vs_outCorner_qs_", 4));
                    vs_inCorner_qsMap.put(o, buildBinaryVar("v" + i + "-" + o.getName() + "-" + sv.getName() + "_vs_inCorner_qs_", 4));
                    auxQuad_vs_inOutCorner_qsMap.put(o, buildBinaryVar("v" + i + "-" + o.getName() + "-" + sv.getName() + "_AuxQuad_vs_inOutCorner_qs_", 2));


                    vs_dOmOn_cqsMap.put(o, vs_dOn_cqsMap);








                    //
                    Map<PseudoBase, GurobiVariable[]> aux_vs_dOutCorner_iqsMap = new HashMap<>();
                    Map<PseudoBase, GurobiVariable> auxQ_vs_dOutCornerMap = new HashMap<>();
                    for (PseudoBase corner: o.getBaseArray()){
                        aux_vs_dOutCorner_iqsMap.put(corner, buildAuxIntVar("aux_vs_dOutCorner_" + corner.getName() + "_iqs_"));
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "auxQ_vs_dOutCorner_" + corner.getName());
                        executor.addVariable(q);
                        auxQ_vs_dOutCornerMap.put(corner, q);


                    }
                    aux_vsdOut_iqsMap.put(o, aux_vs_dOutCorner_iqsMap);
                    auxQ_vsdOutMap.put(o, auxQ_vs_dOutCornerMap);


                }
                vp.vs_relObstacles_qs.put(sv, vs_relObstacles_qsMap);
                vp.vs_relObstaclesD_qs.put(sv, vs_relObstaclesD_qsMap);
                vp.vs_relObstacles_q.put(sv, vs_relObstaclesD_qMap);

                vp.vs_corner_qs.put(sv, vs_corner_qsMap);
                vp.vs_omOnCnn_q.put(sv, vs_omOnCnn_qMap);
                vp.vs_omOnCorner_qs.put(sv, vs_omOnCorner_qsMap);
                vp.auxQuad_vs_omOnCorner_qs.put(sv, auxQuad_vs_omOnCorner_qsMap);
                vp.vs_dOmOn_cqs.put(sv, vs_dOmOn_cqsMap);

                vp.vs_inOutCnn_qs.put(sv, vs_inOutCnn_qsMap);
                vp.vs_outCorner_qs.put(sv, vs_outCorner_qsMap);
                vp.vs_inCorner_qs.put(sv, vs_inCorner_qsMap);
                vp.auxQuad_vs_inOutCorner_qs.put(sv, auxQuad_vs_inOutCorner_qsMap);

                vp.aux_vs_dOutCorner_iqs.put(sv, aux_vsdOut_iqsMap);
                vp.auxQ_vs_dOutCorner.put(sv, auxQ_vsdOutMap);





                /*
                vs_detour_qs
                q_ij^d: detour trigger
                */
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + sv.getName() + "_vs_detour_q");
                executor.addVariable(q);
                vp.vs_detour_q.put(sv, q);

                /*
                Auxiliary absolute values: aux_vsDist_iqs
                0: |vi.x-sj.x|
                1: |vi.y-sj.y|
                2: min(|vi.x-sj.x|,|vi.y-sj.y|)
                3: ||vi.x-sj.x|-|vi.y-sj.y||
                4: vi.x-sj.x
                5: vi.y-sj.y
                6: |vi.x-sj.x|-|vi.y-sj.y|
                 */
                vp.aux_vsDist_iqs.put(sv, buildAuxIntVar("v" + i + ";" + sv.getName() + "_aux_vsDist_iqs_"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + sv.getName() + "_auxQ_vsDist");
                executor.addVariable(q);
                vp.auxQ_vsDist.put(sv, q);

                /*
                vs_dOut_cqs
                0: vs_odOut:MIN
                1: vs_odOut:DIFF
                 */
                vp.vs_dOut_cqs.put(sv, buildIntVar(0, M, "v" + i + ";" + sv.getName() + "_vs_dOut_cqs_", 2));

                /*
                vs_dIn_cqs
                0: vs_odIn:MIN
                1: vs_odIn:DIFF
                 */
                vp.vs_dIn_cqs.put(sv, buildIntVar(0, M, "v" + i + ";" + sv.getName() + "_vs_dIn_cqs_", 2));
                /*
                dvs
                0: dvs:MIN
                1: dvs:DIFF
                 */
                vp.vs_dist_cqs.put(sv, buildIntVar(0, M, "v" + i + ";" + sv.getName() + "_dvs_", 2));

            }


        }


    }

    private GurobiVariable[] buildAuxIntVar(String varName) {
        GurobiVariable[] qs = new GurobiVariable[7];
        for (int i = 0; i < 4; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, 0, M, varName + i);
            executor.addVariable(qs[i]);
        }
        for (int i = 4; i < 7; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, -M, M, varName + i);
            executor.addVariable(qs[i]);
        }
        return qs;
    }


    private GurobiVariable[] buildContinuousVar(int lb, int ub, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.CONTINUOUS, lb, ub, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }

    private GurobiVariable[] buildBinaryVar(String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }

    private GurobiVariable[] buildIntVar(int lb, int ub, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.INTEGER, lb, ub, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }


    public OppositeType relationDetermineRegardOs(ArrayList<Obstacle> relevantObstacles, PseudoBase cur_n, PseudoBase other_n) {

        OppositeType type = OppositeType.NoDetour;
        for (Obstacle cur_o : relevantObstacles) {
            for (Obstacle other_o : relevantObstacles) {
                //topL to bottomR
                if (cur_o.topL_bottomR_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[4] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[7] == 1) {
                    type = OppositeType.ul_lr;
                    break;
                }
                //bottomR to topL
                if (cur_o.bottomR_topL_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[7] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[4] == 1) {
                    type = OppositeType.lr_ul;
                    break;
                }
                //topR to bottomL
                if (cur_o.topR_bottomL_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[5] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[6] == 1) {
                    type = OppositeType.ur_ll;
                    break;
                }
                //bottomL to topR
                if (cur_o.bottomL_topR_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[6] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[5] == 1) {
                    type = OppositeType.ll_ur;
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

        for (Obstacle o : obstacles) {

            basicBinaryVariables(master, o);
            for (PseudoBase slave : slaves) {
                basicBinaryVariables(slave, o);
            }


            /*
            oo_dir
             */
            for (Obstacle other_o : obstacles) {
                if (!other_o.getName().equals(o.getName())) {

                    //OtL:
                    if (o.atL(other_o)) {
                        o.addTotLObstacles(other_o);
                    }

                    //OtR:
                    if (o.atR(other_o)) {
                        o.addTotRObstacles(other_o);
                    }

                    //ObL:
                    if (o.abL(other_o)) {
                        o.addTobLObstacles(other_o);
                    }

                    //ObR:
                    if (o.abR(other_o)) {
                        o.addTobRObstacles(other_o);
                    }

                    //OdL:
                    if (o.odL(other_o)) {
                        o.addTodLObstacles(other_o);
                    }
                    //OdR:
                    if (o.odR(other_o)) {
                        o.addTodRObstacles(other_o);
                    }
                    //OdT:
                    if (o.odT(other_o)) {
                        o.addTodTObstacles(other_o);
                    }
                    //OdB:
                    if (o.odB(other_o)) {
                        o.addTodBObstacles(other_o);
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
    private void basicBinaryVariables(PseudoBase base, Obstacle o) {

        /*
        oDir_q: UL, UR, LR, LL, L, R, T, B
         */
        int[] odir_q = new int[8];
        //UL
        if (base.getY() < base.getX() + o.getMaxY() - o.getMinX()) {
            odir_q[0] = 0;
        } else
            odir_q[0] = 1;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()) {
            odir_q[1] = 0;
        } else
            odir_q[1] = 1;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()) {
            odir_q[2] = 1;
        } else
            odir_q[2] = 0;
        //LL
        if (base.getY() < -base.getX() + o.getMinY() + o.getMinX()) {
            odir_q[3] = 1;
        } else
            odir_q[3] = 0;
        //L
        if (base.getX() < o.getMinX()) {
            odir_q[4] = 0;
        } else
            odir_q[4] = 1;
        //R
        if (base.getX() > o.getMaxX()) {
            odir_q[5] = 0;
        } else
            odir_q[5] = 1;
        //T
        if (base.getY() > o.getMaxY()) {
            odir_q[6] = 0;
        } else
            odir_q[6] = 1;
        //B
        if (base.getY() < o.getMinY()) {
            odir_q[7] = 0;
        } else
            odir_q[7] = 1;
        base.addToPseudo_oDir_qs(o, odir_q);

        /*
        oRel_q:
         */
        int[] orel_q = new int[8];
        //UpperLeft
        if (odir_q[3] + odir_q[1] + odir_q[4] + odir_q[6] == 0) {
            orel_q[0] = 1;
        } else orel_q[0] = 0;
        //UpperRight
        if (odir_q[0] + odir_q[2] + odir_q[5] + odir_q[6] == 0) {
            orel_q[1] = 1;
        } else orel_q[1] = 0;
        //LowerLeft
        if (odir_q[0] + odir_q[2] + odir_q[4] + odir_q[7] == 0) {
            orel_q[2] = 1;
        } else orel_q[2] = 0;
        //LowerRight
        if (odir_q[1] + odir_q[3] + odir_q[5] + odir_q[7] == 0) {
            orel_q[3] = 1;
        } else orel_q[3] = 0;

        //d_Left
        if (odir_q[5] + odir_q[6] + odir_q[7] == 3) {
            orel_q[4] = 1;
        } else orel_q[4] = 0;
        //d_Right
        if (odir_q[4] + odir_q[6] + odir_q[7] == 3) {
            orel_q[5] = 1;
        } else orel_q[5] = 0;
        //d_Top
        if (odir_q[4] + odir_q[5] + odir_q[7] == 3) {
            orel_q[6] = 1;
        } else orel_q[6] = 0;
        //d_Bottom
        if (odir_q[4] + odir_q[5] + odir_q[6] == 3) {
            orel_q[7] = 1;
        } else orel_q[7] = 0;


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


    public String convertGrbIntArrayToString(GurobiVariable[] grbIntArray) throws GRBException {
        StringBuilder grbAsString = new StringBuilder("[");
        for (GurobiVariable v : grbIntArray) {
            grbAsString.append(v.getIntResult() + " ");
        }
        grbAsString.delete(grbAsString.length() - 1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }

    public String convertGrbContArrayToString(GurobiVariable[] grbContArray) throws GRBException {
        StringBuilder grbAsString = new StringBuilder("[");
        for (GurobiVariable v : grbContArray) {
            grbAsString.append(v.getContResult() + " ");
        }
        grbAsString.delete(grbAsString.length() - 1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }

    public String convertObstacleArrayToString(ArrayList<Obstacle> obstacles) {

        StringBuilder arrayAsString = new StringBuilder(":");
        for (Obstacle o : obstacles) {
            arrayAsString.append(o.getName() + "; ");
        }
        arrayAsString.append("||");
        return arrayAsString.toString();
    }

}
