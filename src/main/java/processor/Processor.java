package processor;

import grb.GurobiConstraint;
import grb.GurobiExecutor;
import grb.GurobiQuadConstraint;
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

        //build Gurobi Variables
        ArrayList<VirtualPointVar> vps = new ArrayList<>();
        buildVars(obstacles, vps, slaves, master);
        executor.updateModelWithVars();
        //System.out.println(executor.getVariables().size());







        /*
        Debug
         */
        /*for (Obstacle o : obstacles) {
            System.out.println(o);
        }*/


        return output;
    }

    public void buildCons(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master) {
        double eps = 3;

        GurobiConstraint c;
        GurobiQuadConstraint qc;

        double dTmp;


        for (VirtualPointVar vp : virtualPointVars) {

            for (Obstacle o : obstacles) {
                //nonl
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_Non_qs.get(o)[0], 1.0);
                c.addToLHS(vp.o_vp_Non_qs.get(o)[1], 1.0);
                c.addToLHS(vp.o_vp_Non_qs.get(o)[2], 1.0);
                c.addToLHS(vp.o_vp_Non_qs.get(o)[3], 1.0);
                c.setSense('=');
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                //nonl.1
                c = new GurobiConstraint();
                c.addToLHS(vp.x, 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Non_qs.get(o)[0], -M);
                c.setRHSConstant(o.getMinX() - eps + M);
                executor.addConstraint(c);
                //nonl.r
                c = new GurobiConstraint();
                c.addToLHS(vp.x, 1.0);
                c.setSense('>');
                c.addToRHS(vp.o_vp_Non_qs.get(o)[1], M);
                c.setRHSConstant(o.getMaxX() + eps - M);
                executor.addConstraint(c);
                //nonl.t
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.o_vp_Non_qs.get(o)[2], M);
                c.setRHSConstant(o.getMaxY() + eps - M);
                executor.addConstraint(c);
                //nonl.b
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Non_qs.get(o)[3], -M);
                c.setRHSConstant(o.getMinY() - eps + M);
                executor.addConstraint(c);


                //diagonal Sets
                //rel.ul
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[0], -M);
                c.setRHSConstant(o.getMaxY() - o.getMinX() + eps);
                executor.addConstraint(c);
                //rel.ur
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[1], -M);
                c.setRHSConstant(o.getMaxY() + o.getMaxX() + eps);
                executor.addConstraint(c);
                //rel.lr
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[2], -M);
                c.setRHSConstant(o.getMinY() - o.getMaxX() + eps);
                executor.addConstraint(c);
                //rel.ll
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[3], -M);
                c.setRHSConstant(o.getMinY() + o.getMinX() + eps);
                executor.addConstraint(c);
                //rel.d
                dTmp = (double) (o.getMinY() - o.getMaxY())/ (double) (o.getMaxX() - o.getMinX());
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, dTmp);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[4], -M);
                c.setRHSConstant(dTmp * o.getMinX() + o.getMaxY() + eps);
                executor.addConstraint(c);
                //rel.u
                dTmp = (double)(o.getMaxY() - o.getMinY()) /(double)(o.getMaxX() - o.getMinX());
                c = new GurobiConstraint();
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, dTmp);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[5], -M);
                c.setRHSConstant(dTmp * o.getMinX() + o.getMinY() + eps);
                executor.addConstraint(c);

                //tL
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[1], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[3], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[5], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[1], 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[3], -1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[5], -1.0);
                executor.addConstraint(c);

                //tR
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[0], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[2], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[4], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[0], 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[2], -1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[4], -1.0);
                executor.addConstraint(c);

                //bL
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[0], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[4], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[2], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[0], 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[4], 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[2], -1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //bR
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[1], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[5], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[3], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.o_vp_diagonalSets_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[1], 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[5], 1.0);
                c.addToRHS(vp.o_vp_Rel_qs.get(o)[3], -1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);







            }


        }


    }


    public void buildVars(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master) {
        GurobiVariable[] qs;
        GurobiVariable q, iq, cq;

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
            virtualPointVars.add(vp);
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
                vp.o_vp_Non_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_o_vp_Non_qs_", 4));

                /*
                o_vp_Rel_qs
                0: ul
                1: ur
                2: lr
                3: ll
                4: d
                5: u
                 */
                vp.o_vp_Rel_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_o_vp_Rel_qs_", 6));

                /*
                o_vp_diagonalSets_qs
                0: o_tL
                1: o_tR
                2: o_bL
                3: o_bR
                 */
                vp.o_vp_diagonalSets_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_o_vp_diagonalSets_qs_", 4));

                /*
                o_vp_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: relative obstacle: aux.2
                 */
                vp.o_vp_relObstacles_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_o_vp_relObstacles_qs_", 5));


                /*
                o_vvCorner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                vp.ovv_Corner_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_o_vvCorner_qs_", 4));

                /*
                o_vv_inOutCnn_qs
                0: ->
                1: <-
                */
                vp.ovv_inOutCnn_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_vv_inOutCnn_qs_", 2));


                /*
                o_vv_oCoordinate_iqs
                0: x_m
                1: y_m
                 */
                vp.ovv_oCoordinate_iqs.put(o, buildIntVar(lb, ub, "v_" + i + ";" + o.getName() + "_vv_oCoordinate_iqs_", 2));



                /*
                vv_ooCnn_q
                0: q_ij^m->n
                 */
                Map<Obstacle, GurobiVariable> vv_ooCnn_qsMap = new HashMap<>();
                /*
                vv_ooDist_cqs
                0: d_m->n
                 */
                Map<Obstacle, GurobiVariable[]> vv_ooDist_cqsMap = new HashMap<>();
                for (Obstacle other_o : obstacles) {

                    GurobiVariable vv_ooCnn_q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + "-" + other_o.getName() + "_vv_ooCnn_q");
                    executor.addVariable(vv_ooCnn_q);
                    vv_ooCnn_qsMap.put(other_o, vv_ooCnn_q);

                    vv_ooDist_cqsMap.put(other_o, buildContinuousVar(0, M, "v_" + i + ";" + o.getName() + "->" + other_o.getName() + "_vv_ooDist_cqs_", 1));
                }
                vp.vv_ooCnn_q.put(o, vv_ooCnn_qsMap);
                vp.vv_ooDist_cqs.put(o, vv_ooDist_cqsMap);

            }

            /*
            vp_detour_qs
            0: ul->lr
            1: lr->ul
            2: ur->ll
            3: ll->ur
            4: detour trigger: aux.3
             */
            vp.vvDetour_qs = buildBinaryVar("v" + i + "_vp_detour_qs_", 5);

            /*
            vv_inOutDist_cqs
            0: d_->
            1: d_<-
             */
            vp.vv_inOutDist_cqs = buildContinuousVar(0, M, "v" + i + "vv_inOutDist_cqs", 2);

            /*
            dist_cqs
            0: vv dist
            1: v.corrS dist
            2: vm dist (only for 1st vp)
             */
            if (i == 0) {
                vp.dist_cqs = buildContinuousVar(0, M, "v" + i + "dist_cqs", 3);
            } else {
                vp.dist_cqs = buildContinuousVar(0, M, "v" + i + "dist_cqs", 2);
            }

            /*
            Regarding Slaves
             */
            for (PseudoBase sv : slaves) {
                //vsCnn_q
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_vsCnn_q_" + sv.getName());
                executor.addVariable(q);
                vp.vsCnn_q.put(sv, q);

                //vsDist_cq
                cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + "vsDist_cq" + sv.getName());
                executor.addVariable(cq);
                vp.vsDist_cq.put(sv, cq);

                /*
                ovs_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: relative obstacle: aux.2
                */
                Map<Obstacle, GurobiVariable[]> ovs_relObstacle_qs = new HashMap<>();
                /*
                ovs_Corner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                Map<Obstacle, GurobiVariable[]> ovs_Corner_qs = new HashMap<>();
                /*
                vs_ooCnn_q
                q_i_sj^m->n
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable>> vs_ooCnn_q = new HashMap<>();
                /*
                ovs_inCnn_q
                sj<-
                 */
                Map<Obstacle, GurobiVariable> ovs_outCnn_qs = new HashMap<>();
                /*
                ovs_oCoordinate_iqs
                0: x_m
                1: y_m
                */
                Map<Obstacle, GurobiVariable[]> ovs_oCoordinate_iqs = new HashMap<>();
                /*
                vs_ooDist_cqs
                0: d_m->n
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vs_ooDist_cqs = new HashMap<>();


                for (Obstacle o : obstacles) {
                    ovs_relObstacle_qs.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_ovs_relObstacles_qs_", 5));
                    ovs_Corner_qs.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_ovs_Corner_qs_", 4));


                    Map<Obstacle, GurobiVariable> vs_ooCnn_qMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> vs_ooDist_cqsMap = new HashMap<>();
                    for (Obstacle other_o : obstacles) {
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_ooCnn_q");
                        executor.addVariable(q);
                        vs_ooCnn_qMap.put(other_o, q);

                        vs_ooDist_cqsMap.put(other_o, buildContinuousVar(0, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_ooDist_cqs_", 1));
                    }
                    vs_ooCnn_q.put(o, vs_ooCnn_qMap);

                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_ovs_inCnn_q");
                    executor.addVariable(q);
                    ovs_outCnn_qs.put(o, q);

                    ovs_oCoordinate_iqs.put(o, buildIntVar(lb, ub, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_ovs_oCoordinate_iqs_", 2));
                    vs_ooDist_cqs.put(o, vs_ooDist_cqsMap);
                }
                vp.ovs_relObstacles_qs.put(sv, ovs_relObstacle_qs);
                vp.ovs_Corner_qs.put(sv, ovs_Corner_qs);
                vp.vs_ooCnn_q.put(sv, vs_ooCnn_q);
                vp.ovs_inCnn_q.put(sv, ovs_outCnn_qs);
                vp.ovs_oCoordinate_iqs.put(sv, ovs_oCoordinate_iqs);
                vp.vs_ooDist_cqs.put(sv, vs_ooDist_cqs);


                /*
                vs_detour_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: q_ij^d: detour trigger: aux.3
                */
                vp.vsDetour_qs.put(sv, buildBinaryVar("v" + i + ";" + sv.getName() + "_vs_detour_qs_", 5));

                /*
                vs_inDist_cqs
                0: d_<-
                 */
                vp.vs_inDist_cqs.put(sv, buildContinuousVar(0, M, "v" + i + ";" + ";" + sv.getName() + "_vs_inDist_cqs_", 1));


            }

        }


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
