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
    o_vp_Non_qs
    0: nonL
    1: nonR
    2: nonT
    3: nonB
     */
    public Map<Obstacle, GurobiVariable[]> o_vp_Non_qs;//binary variables for nonoverlapping

    /*
    o_vp_Rel_qs
    0: ul
    1: ur
    2: lr
    3: ll
    4: d
    5: u
     */
    public Map<Obstacle, GurobiVariable[]> o_vp_Rel_qs;//binary variables for (rel.ul) -- (rel.u)

    /*
    o_vp_diagonalSets_qs
    0: o_tL
    1: o_tR
    2: o_bL
    3: o_bR
     */
    public Map<Obstacle, GurobiVariable[]> o_vp_diagonalSets_qs;//binary variables for (tL) -- (bR)

    /*
    o_vp_relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: relative obstacle: aux.1/cnn.4
     */
    public Map<Obstacle, GurobiVariable[]> o_vp_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vp_detour_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: q_ij^d: detour trigger: aux.2/cnn.4
     */
    public GurobiVariable[] vvDetour_qs;//binary variables regarding next virtualPoint for (ul->lr.2) -- (ll->ur.2)



    /*
    Path Length Computation regarding Next VirtualPoint
     */

    /*
    ovv_Corner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<Obstacle, GurobiVariable[]> ovv_Corner_qs;

    /*
    vv_ooCnn_q
    q_i_vj^m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> vv_ooCnn_q;//binaryVar regarding next virtualPoint for relObstacles' connection

    /*
    ovv_inOutCnn_qs
    0: vi->
    1: vi<-
     */
    public Map<Obstacle, GurobiVariable[]> ovv_inOutCnn_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    /*
    ovv_oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> ovv_oCoordinate_iqs;//intVar regarding next virtualPoint: coordinates of the selected intermedia point

    /*
    vv_ooDist_cqs
    0: d_m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vv_ooDist_cqs;//intVar: path length between o_m and o_n

    /*
    vv_inOutDist_cqs
    0: d_->
    1: d_<-
     */
    public GurobiVariable[] vv_inOutDist_cqs;//contVar

    /*
    Auxiliary absolute values
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, GurobiVariable[]> aux_ovv_inOutDist_cqs;

    /*
    Auxiliary absolute values
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_vv_ooCnn_cqs;

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
    public GurobiVariable[] aux_vvDist_cqs;

    /*
    Detour triggering regarding Slaves
     */
    //vsCnn_q
    public Map<PseudoBase, GurobiVariable> vsCnn_q;//binaryVar
    
    //vsDist_cq
    public Map<PseudoBase, GurobiVariable> vsDist_cq;

    /*
    o_vs_relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: relative obstacle: aux.2
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> ovs_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vs_detour_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: q_ij^d: detour trigger: aux.3
     */
    public Map<PseudoBase, GurobiVariable[]> vsDetour_qs;//binary variables regarding Slaves for (ul->lr.2) -- (ll->ur.2)



    /*
    Path Length Computation regarding Slaves
     */
    /*
    ovs_Corner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> ovs_Corner_qs;

    /*
    vs_ooCnn_q
    q_i_sj^m->n
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable>>> vs_ooCnn_q;//binaryVar regarding Slaves for relObstacles' connection

    /*
    ovs_inCnn_q
    sj<-
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable>> ovs_inCnn_q;//binaryVar regarding Slaves for starting and end point connection

    /*
    ovs_oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> ovs_oCoordinate_iqs;//intVar regarding Slaves: coordinates of the selected intermedia point

    /*
    vs_ooDist_cqs
    0: d_m->n
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> vs_ooDist_cqs;//contVar: path length between o_m and o_n

    /*
    vs_inDist_cqs
    0: d_<-
     */
    public Map<PseudoBase, GurobiVariable[]> vs_inDist_cqs;//contVar

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

        this.o_vp_Non_qs = new HashMap<>();
        this.o_vp_Rel_qs = new HashMap<>();
        this.o_vp_diagonalSets_qs = new HashMap<>();
        this.o_vp_relObstacles_qs = new HashMap<>();


        this.ovv_Corner_qs = new HashMap<>();
        this.vv_ooCnn_q = new HashMap<>();
        this.ovv_inOutCnn_qs = new HashMap<>();
        this.ovv_oCoordinate_iqs = new HashMap<>();
        this.vv_ooDist_cqs = new HashMap<>();
        this.aux_ovv_inOutDist_cqs = new HashMap<>();
        this.aux_vv_ooCnn_cqs = new HashMap<>();

        this.vsCnn_q = new HashMap<>();
        this.vsDist_cq = new HashMap<>();
        this.ovs_relObstacles_qs = new HashMap<>();
        this.vsDetour_qs = new HashMap<>();
        this.ovs_Corner_qs = new HashMap<>();
        this.vs_ooCnn_q = new HashMap<>();
        this.ovs_inCnn_q = new HashMap<>();
        this.ovs_oCoordinate_iqs = new HashMap<>();
        this.vs_ooDist_cqs = new HashMap<>();
        this.vs_inDist_cqs = new HashMap<>();

    }
}
