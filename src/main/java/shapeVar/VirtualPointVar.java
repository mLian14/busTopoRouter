package shapeVar;

import grb.GurobiVariable;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 09.04.23
 */
public class VirtualPointVar {

    public GurobiVariable x;//x-coordinate of v_point
    public GurobiVariable y;//y-coordinate of v_point


    /*
    Detour triggering regarding NEXT VirtualPoint
     */

    /*
    Non-overlapping
    0: nonL
    1: nonR
    2: nonT
    3: nonB
     */
    public Map<Obstacle, GurobiVariable[]> non_qs;//binary variables for nonoverlapping

    /*
    rel_qs
    0: ul
    1: ur
    2: lr
    3: ll
    4: d
    5: u
     */
    public Map<Obstacle, GurobiVariable[]> rel_qs;//binary variables for (rel.ul) -- (rel.u)

    /*
    diagonal_qs
    0: o_tL
    1: o_tR
    2: o_bL
    3: o_bR
     */
    public Map<Obstacle, GurobiVariable[]> diagonal_qs;//binary variables for (tL) -- (bR)

    /*
    relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: relative obstacle: aux.1/cnn.4
     */
    public Map<Obstacle, GurobiVariable[]> relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    detour_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: q_ij^d: detour trigger: aux.2/cnn.4
     */
    public GurobiVariable[] detour_qs;//binary variables regarding next virtualPoint for (ul->lr.2) -- (ll->ur.2)



    /*
    Path Length Computation regarding Next VirtualPoint
     */

    /*
    corner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<Obstacle, GurobiVariable[]> corner_qs;

    /*
    omOnCnn_q
    q_i_vj^m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> omOnCnn_q;//binaryVar regarding next virtualPoint for relObstacles' connection

    /*
    inOutCnn_qs
    0: vi->
    1: vj<-
     */
    public Map<Obstacle, GurobiVariable[]> inOutCnn_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    /*
    oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> oCoordinate_iqs;//intVar regarding next virtualPoint: coordinates of the selected intermedia point

    /*
    dOmOn_cq
    0: d_m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> dOmOn_cq;//intVar: path length between o_m and o_n
    /*
    Auxiliary absolute values: aux_dOmOn_cqs
    0: |x_m - x_n|
    1: |y_m - y_n|
    2: min(|x_m - x_n|, |y_m - y_n|)
    3: ||x_m - x_n| - |y_m - y_n||
    4: x_m - x_n
    5: y_m - y_n
    6: |x_m - x_n| - |y_m - y_n|
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_dOmOn_cqs;


    /*
    dInOut_cqs
    0: d_->
    1: d_<-
     */
    public GurobiVariable[] dInOut_cqs;//contVar
    /*
    Auxiliary absolute values: aux_dOut_cqs
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, GurobiVariable[]> aux_dOut_cqs;
    /*
    Auxiliary absolute values: aux_dIn_cqs
    0: |vj.x - x_n|
    1: |vj.y - y_n|
    2: min(|vj.x - x_n|, |vj.y - y_n|)
    3: ||vj.x - x_n| - |vj.y - y_n||
    4: vj.x - x_n
    5: vj.y - y_n
    6: |vj.x - x_n| - |vj.y - y_n|
     */
    public Map<Obstacle, GurobiVariable[]> aux_dIn_cqs;

    /*
    dist_cqs
    0: vv dist: d_ij
    1: v.corrS dist
    2: vm dist (only for 1st vp)
     */
    public GurobiVariable[] dist_cqs;

    /*
    Auxiliary absolute values: vv
    0: |vi.x - vj.x|
    1: |vi.y - vj.y|
    2: min(|vi.x - vj.x|, |vi.y - vj.y|)
    3: ||vi.x - vj.x| - |vi.y - vj.y||
    4: vi.x - vj.x
    5: vi.y - vj.y
    6: |vi.x - vj.x| - |vi.y - vj.y|
     */
    public GurobiVariable[] aux_dist_cqs;







    /*
    Detour triggering regarding Slaves
     */
    //vsCnn_q
    public Map<PseudoBase, GurobiVariable> vsCnn_q;//binaryVar


    /*
    vs_relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: relative obstacle: aux.2
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vs_detour_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: q_ij^d: detour trigger: aux.3
     */
    public Map<PseudoBase, GurobiVariable[]> vs_detour_qs;//binary variables regarding Slaves for (ul->lr.2) -- (ll->ur.2)



    /*
    Path Length Computation regarding Slaves
     */
    /*
    vs_corner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_corner_qs;

    /*
    vs_omOnCnn_q
    q_i_sj^m->n
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable>>> vs_omOnCnn_q;//binaryVar regarding Slaves for relObstacles' connection

    /*
    vs_inCnn_q
    sj<-
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable>> vs_inCnn_q;//binaryVar regarding Slaves for starting and end point connection

    /*
    vs_oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_oCoordinate_iqs;//intVar regarding Slaves: coordinates of the selected intermedia point

    /*
    vs_dOmOn_cqs
    0: d_m->n
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> vs_dOmOn_cqs;//contVar: path length between o_m and o_n

    /*
    vs_dIn_cqs
    0: d_<-
     */
    public Map<PseudoBase, GurobiVariable[]> vs_dIn_cqs;//contVar

    /*
    d_i_sj
     */
    public Map<PseudoBase, GurobiVariable> vs_dist_cq;


    /*
    Auxiliary absolute values: vs
    0: |vi.x - sj.x|
    1: |vi.y - sj.y|
    2: min(|vi.x - sj.x|, |vi.y - sj.y|)
    3: ||vi.x - sj.x| - |vi.y - sj.y||
    4: vi.x - sj.x
    5: vi.y - sj.y
    6: |vi.x - sj.x| - |vi.y - sj.y|
     */
    public Map<PseudoBase, GurobiVariable[]> aux_vsDist_cqs;


    public VirtualPointVar() {

        this.non_qs = new HashMap<>();
        this.rel_qs = new HashMap<>();
        this.diagonal_qs = new HashMap<>();
        this.relObstacles_qs = new HashMap<>();


        this.corner_qs = new HashMap<>();
        this.omOnCnn_q = new HashMap<>();
        this.inOutCnn_qs = new HashMap<>();
        this.oCoordinate_iqs = new HashMap<>();
        this.dOmOn_cq = new HashMap<>();
        this.aux_dOut_cqs = new HashMap<>();
        this.aux_dOmOn_cqs = new HashMap<>();
        this.aux_dIn_cqs = new HashMap<>();


        this.vsCnn_q = new HashMap<>();
        this.vs_relObstacles_qs = new HashMap<>();
        this.vs_detour_qs = new HashMap<>();
        this.vs_corner_qs = new HashMap<>();
        this.vs_omOnCnn_q = new HashMap<>();
        this.vs_inCnn_q = new HashMap<>();
        this.vs_oCoordinate_iqs = new HashMap<>();
        this.vs_dOmOn_cqs = new HashMap<>();
        this.vs_dIn_cqs = new HashMap<>();
        this.aux_vsDist_cqs = new HashMap<>();
        this.vs_dist_cq = new HashMap<>();

    }
}
