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
    Connection with Master
    VVVV
     */

    /*
    VM:relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: relative obstacle: aux.1/cnn.4
     */
    public Map<Obstacle, GurobiVariable[]> vm_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)
    /*
    VM:detour_qs
    q_ij^d: detour trigger: aux.2/cnn.4
     */
    public GurobiVariable vm_detour_q;//binary variables regarding next virtualPoint for (ul->lr.2) -- (ll->ur.2)

    /*
    VM:corner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<Obstacle, GurobiVariable[]> vm_corner_qs;

    /*
    VM:omOnCnn_q
    q_i_vj^m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> vm_omOnCnn_q;//binaryVar regarding next virtualPoint for relObstacles' connection

    /*
    VM:inOutCnn_qs
    0: vi->
    1: vj<-
     */
    public Map<Obstacle, GurobiVariable[]> vm_inOutCnn_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    /*
    VM:oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> vm_oCoordinate_iqs;//intVar regarding next virtualPoint: coordinates of the selected intermedia point


    /*
    VM: dOut_cqs
    0: d_i_ms->: Min
    1: d_i_ms->: Diff
     */
    public Map<Obstacle, GurobiVariable[]> vm_odOut_cqs;
    /*
    VM: dIn_cqs
    0: d_i_ms<-: Min
    1: d_i_ms<-: Diff
     */
    public Map<Obstacle, GurobiVariable[]> vm_odIn_cqs;

    /*
    Auxiliary absolute values: aux_vmdOut_iqs
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, GurobiVariable[]> aux_vmdOut_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_vmdOut;
    /*
    Auxiliary absolute values: aux_vmdIn_iqs
    0: |x_n - ms.x|
    1: |y_n - ms.y|
    2: min(|x_n - ms.x|, |y_n - ms.y|)
    3: ||x_n - ms.x| - |y_n - ms.y||
    4: x_n - ms.x
    5: y_n - ms.y
    6: |x_n - ms.x| - |y_n - ms.y|
     */
    public Map<Obstacle, GurobiVariable[]> aux_vmdIn_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_vmdIn;



    /*
    VM:dOmOn_cqs
    0: d_m->n:MIN
    1: d_m->n:DIFF
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vm_dOmOn_cqs;//intVar: path length between o_m and o_n


    /*
    Auxiliary absolute values: aux_vmdOmOn_iqs
    0: |x_m - x_n|
    1: |y_m - y_n|
    2: min(|x_m - x_n|, |y_m - y_n|)
    3: ||x_m - x_n| - |y_m - y_n||
    4: x_m - x_n
    5: y_m - y_n
    6: |x_m - x_n| - |y_m - y_n|
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_vmdOmOn_iqs;
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> auxQ_vmdOmOn;

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
    public GurobiVariable[] aux_vmDist_iqs;
    public GurobiVariable auxQ_vmDist;


    /*
    vm_dist_cqs:
    0: vm_dist: MIN
    1: vm_dist: DIFF
     */
    public GurobiVariable[] vm_dist_cqs;

    /*
    AAAA
    Connection with Master
     */




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
    public Map<Obstacle, GurobiVariable[]> non_qs;//binary variables for non-overlapping

    /*
    dir_qs
    0: ul
    1: ur
    2: lr
    3: ll
     */
    public Map<Obstacle, GurobiVariable[]> dir_qs;//binary variables for (rel.ul) -- (rel.u)

    /*
    rel_qs
    0: o_tL
    1: o_tR
    2: o_bL
    3: o_bR
     */
    public Map<Obstacle, GurobiVariable[]> rel_qs;//binary variables for (tL) -- (bR)
    /*
    relD_qs
    0: o_L
    1: o_R
    2: o_T
    3: o_B
     */
    public Map<Obstacle, GurobiVariable[]> relD_qs;//binary variables for (L) (R) (T) (B)


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
    relObstaclesD_qs
    0: l->r
    1: r->l
    2: t->b
    3: b->t
     */
    public Map<Obstacle, GurobiVariable[]> relObstaclesD_qs;
    /*
    indicate the relevance of each obstacle
     */
    public Map<Obstacle, GurobiVariable> relObstacles_q;



    /*
    detour_qs
    q_ij^d: detour trigger: aux.2/cnn.4
     */
    public GurobiVariable detour_q;//binary variables regarding next virtualPoint for (ul->lr.2) -- (ll->ur.2)



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
    oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> oCoordinate_iqs;//intVar regarding next virtualPoint: coordinates of the selected intermedia point


    /*
    inOutCnn_qs
    0: vi->
    1: vj<-
     */
    public Map<Obstacle, GurobiVariable[]> inOutCnn_qs;//binaryVar regarding next virtualPoint for starting and end point connection




    /*
    dOut_cqs
    0: d_->:MIN
    1: d_->:DIFF
     */
    public Map<Obstacle, GurobiVariable[]> odOut_cqs;
    /*
    dIn_cqs
    0: d_<-:MIN
    1: d_<-:DIFF
     */
    public Map<Obstacle, GurobiVariable[]> odIn_cqs;


    /*
    Auxiliary absolute values: aux_dOut_iqs
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, GurobiVariable[]> aux_dOut_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_dOut;
    /*
    Auxiliary absolute values: aux_dIn_iqs
    0: |vj.x - x_n|
    1: |vj.y - y_n|
    2: min(|vj.x - x_n|, |vj.y - y_n|)
    3: ||vj.x - x_n| - |vj.y - y_n||
    4: vj.x - x_n
    5: vj.y - y_n
    6: |vj.x - x_n| - |vj.y - y_n|
     */
    public Map<Obstacle, GurobiVariable[]> aux_dIn_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_dIn;


    /*
    dOmOn_cqs
    0: d_m->n:MIN
    1: d_m->n:DIFF
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> dOmOn_cqs;


    /*
    Auxiliary absolute values: aux_dOmOn_iqs
    0: |x_m - x_n|
    1: |y_m - y_n|
    2: min(|x_m - x_n|, |y_m - y_n|)
    3: ||x_m - x_n| - |y_m - y_n||
    4: x_m - x_n
    5: y_m - y_n
    6: |x_m - x_n| - |y_m - y_n|
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_dOmOn_iqs;
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> auxQ_dOmOn;


    /*
    dist_cqs: vv_dist
    0: dvv:MIN
    1: dvv:DIFF
     */
    public GurobiVariable[] dist_cqs;


    /*
    d_vCorS: dist between virtualPoint and corresponding slave
    0: d_vCorS:MIN
    1: d_vCors:DIFF
    todo
     */
    public GurobiVariable[] vs_corDist_cqs;





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
    public GurobiVariable[] aux_dist_iqs;
    public GurobiVariable auxQ_dist;







    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
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
    q_ij^d: detour trigger: aux.3
     */
    public Map<PseudoBase, GurobiVariable> vs_detour_q;//binary variables regarding Slaves for (ul->lr.2) -- (ll->ur.2)



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
    vs_dOmOn_cqs
    0: vs_d_m->n:MIN
    1: vs_d_m->n:DIFF
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> vs_dOmOn_cqs;


    /*
    vs_inOutCnn_qs
    0: vi_s->
    1: sj<-
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_inOutCnn_qs;//binaryVar regarding Slaves for starting and end point connection

    /*
    vs_odOut_cqs
    0: vs_odOut:MIN
    1: vs_odOut:DIFF
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_odOut_cqs;

    /*
    vs_odIn_cqs
    0: vs_odIn:MIN
    1: vs_odIn:DIFF
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_odIn_cqs;

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
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> aux_vsdOut_iqs;
    public Map<PseudoBase, Map<Obstacle, GurobiVariable>> auxQ_vsdOut;

    /*
    Auxiliary absolute values: aux_vsdIn_iqs
    0: |x_m - sj.x|
    1: |y_m - sj.y|
    2: min(|x_m - sj.x|, |y_m - sj.y|)
    3: ||x_m - sj.x| - |y_m - sj.y||
    4: x_m - sj.x
    5: y_m - sj.y
    6: |x_m - sj.x| - |vi.y - sj.y|
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> aux_vsdIn_iqs;
    public Map<PseudoBase, Map<Obstacle, GurobiVariable>> auxQ_vsdIn;



    /*
    vs_oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_oCoordinate_iqs;//intVar regarding Slaves: coordinates of the selected intermedia point


    /*
    Auxiliary absolute values: aux_vsdOmOn_iqs
    0: |x_m - x_n|
    1: |y_m - y_n|
    2: min(|x_m - x_n|, |y_m - y_n|)
    3: ||x_m - x_n| - |y_m - y_n||
    4: x_m - x_n
    5: y_m - y_n
    6: |x_m - x_n| - |y_m - y_n|
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> aux_vsdOmOn_iqs;
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable>>> auxQ_vsdOmOn;






    /*
    dvs
    0: dvs:MIN
    1: dvs:DIFF
     */
    public Map<PseudoBase, GurobiVariable[]> vs_dist_cqs;



    /*
    Auxiliary absolute values: vsDist
    0: |vi.x - sj.x|
    1: |vi.y - sj.y|
    2: min(|vi.x - sj.x|, |vi.y - sj.y|)
    3: ||vi.x - sj.x| - |vi.y - sj.y||
    4: vi.x - sj.x
    5: vi.y - sj.y
    6: |vi.x - sj.x| - |vi.y - sj.y|
     */
    public Map<PseudoBase, GurobiVariable[]> aux_vsDist_iqs;
    public Map<PseudoBase, GurobiVariable> auxQ_vsDist;


    public VirtualPointVar() {

        this.non_qs = new HashMap<>();
        this.dir_qs = new HashMap<>();
        this.rel_qs = new HashMap<>();
        this.relD_qs = new HashMap<>();
        this.relObstacles_qs = new HashMap<>();
        this.relObstaclesD_qs = new HashMap<>();
        this.relObstacles_q = new HashMap<>();


        this.corner_qs = new HashMap<>();
        this.omOnCnn_q = new HashMap<>();
        this.inOutCnn_qs = new HashMap<>();
        this.oCoordinate_iqs = new HashMap<>();
        this.aux_dOut_iqs = new HashMap<>();
        this.aux_dOmOn_iqs = new HashMap<>();
        this.aux_dIn_iqs = new HashMap<>();
        this.aux_vsdOmOn_iqs = new HashMap<>();


        this.vsCnn_q = new HashMap<>();


        this.vs_relObstacles_qs = new HashMap<>();
        this.vs_detour_q = new HashMap<>();
        this.vs_corner_qs = new HashMap<>();
        this.vs_omOnCnn_q = new HashMap<>();
        this.vs_inOutCnn_qs = new HashMap<>();
        this.vs_oCoordinate_iqs = new HashMap<>();
        this.aux_vsDist_iqs = new HashMap<>();
        this.aux_vsdIn_iqs = new HashMap<>();
        this.aux_vsdOut_iqs = new HashMap<>();


        this.auxQ_dOmOn = new HashMap<>();
        this.auxQ_dOut = new HashMap<>();
        this.auxQ_dIn = new HashMap<>();
        this.auxQ_vsdOut = new HashMap<>();
        this.auxQ_vsdIn = new HashMap<>();
        this.auxQ_vsdOmOn = new HashMap<>();
        this.auxQ_vsDist = new HashMap<>();

        this.vm_relObstacles_qs = new HashMap<>();
        this.vm_corner_qs = new HashMap<>();
        this.vm_omOnCnn_q = new HashMap<>();
        this.vm_inOutCnn_qs = new HashMap<>();
        this.vm_oCoordinate_iqs = new HashMap<>();

        this.aux_vmdOut_iqs = new HashMap<>();
        this.auxQ_vmdOut = new HashMap<>();
        this.aux_vmdIn_iqs = new HashMap<>();
        this.auxQ_vmdIn = new HashMap<>();
        this.aux_vmdOmOn_iqs = new HashMap<>();
        this.auxQ_vmdOmOn = new HashMap<>();


        this.vm_odIn_cqs = new HashMap<>();
        this.vm_odOut_cqs = new HashMap<>();
        this.vm_dOmOn_cqs = new HashMap<>();

        this.odOut_cqs = new HashMap<>();
        this.odIn_cqs = new HashMap<>();
        this.dOmOn_cqs = new HashMap<>();

        this.vs_dOmOn_cqs = new HashMap<>();
        this.vs_odOut_cqs = new HashMap<>();
        this.vs_odIn_cqs = new HashMap<>();

        this.vs_dist_cqs = new HashMap<>();

    }
}
