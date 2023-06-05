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
    vm:relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: relative obstacle: aux.1/cnn.4
     */
    public Map<Obstacle, GurobiVariable[]> vm_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vm:relObstaclesD_qs
    0: l->r
    1: r->l
    2: t->b
    3: b->t
     */
    public Map<Obstacle, GurobiVariable[]> vm_relObstaclesD_qs;

    /*
    indicate relevant obstacles
     */
    public Map<Obstacle, GurobiVariable> vm_relObstacles_q;


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
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vm_omOnCorner_qs;
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> auxQuad_vm_omOnCorner_qs;

    /*
    VM:inOutCnn_qs
    0: vi->
    1: ->ms
     */
    public Map<Obstacle, GurobiVariable[]> vm_inOutCnn_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    /*
    vm_outCorner_qs
    0: vi->o.ll
    1: vi->o.ul
    2: vi->o.ur
    3: vi->o.lr
     */
    public Map<Obstacle, GurobiVariable[]> vm_outCorner_qs;
    /*
    vm_inCorner_qs
    0: o.ll->vi+1
    1: o.ul->vi+1
    2: o.ur->vi+1
    3: o.lr->vi+1
     */
    public Map<Obstacle, GurobiVariable[]> vm_inCorner_qs;
    public Map<Obstacle, GurobiVariable[]> auxQuad_vm_inOutCorner_qs;

    /*
    vm_dOut_cqs
    0: v0->: min
    1: v0->: xy
     */
    public GurobiVariable[] vm_dOut_cqs;
    /*
    vm_dIn_cqs
    0: ->ms: Min
    1: ->ms: Diff
     */
    public GurobiVariable[] vm_dIn_cqs;

    /*
    Auxiliary absolute values: aux_dOutCorner_iqs
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, Map<PseudoBase, GurobiVariable[]>> aux_vm_dOutCorner_iqs;
    public Map<Obstacle, Map<PseudoBase, GurobiVariable>> auxQ_vm_dOutCorner;




    /*
    VM:dOmOn_cqs
    0: d_m->n:MIN
    1: d_m->n:DIFF
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vm_dOmOn_cqs;//intVar: path length between o_m and o_n

    /*
    vm_dist_cqs:
    0: vm_dist: MIN
    1: vm_dist: DIFF
     */
    public GurobiVariable[] vm_dist_cqs;

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
    AAAA
    Connection with Master
     */



    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
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
    q_ij^d: detour trigger: cnnRules.5, 6,
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
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> omOnCorner_qs;
    /*
    0: cnnRules.3
    1: cnnRules.4
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> auxQuad_omOnCorner_qs;


    /*
    inOutCnn_qs
    0: vi->
    1: ->pj
     */
    public Map<Obstacle, GurobiVariable[]> inOutCnn_qs;//binaryVar regarding next virtualPoint for starting and end point connection
    /*
    outCorner_qs
    0: vi->o.ll
    1: vi->o.ul
    2: vi->o.ur
    3: vi->o.lr
     */
    public Map<Obstacle, GurobiVariable[]> outCorner_qs;
    /*
    inCorner_qs
    0: o.ll->vi+1
    1: o.ul->vi+1
    2: o.ur->vi+1
    3: o.lr->vi+1
     */
    public Map<Obstacle, GurobiVariable[]> inCorner_qs;

    /*
    0: cnnRules.5
    1: cnnRules.6
     */
    public Map<Obstacle, GurobiVariable[]> auxQuad_inOutCorners_qs;

    /*
    dOut_cqs
    0: dvi->:MIN
    1: dvi->:DIFF
     */
    public GurobiVariable[] dOut_cqs;
    /*
    dIn_cqs
    0: d->pj:MIN
    1: d->pj:DIFF
     */
    public GurobiVariable[] dIn_cqs;



    /*
    Auxiliary absolute values: aux_dOutCorner_iqs
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, Map<PseudoBase, GurobiVariable[]>> aux_dOutCorner_iqs;
    public Map<Obstacle, Map<PseudoBase, GurobiVariable>> auxQ_dOutCorner;


    /*
    Auxiliary absolute values: aux_dInCorner_iqs
    0: |vj.x - x_n|
    1: |vj.y - y_n|
    2: min(|vj.x - x_n|, |vj.y - y_n|)
    3: ||vj.x - x_n| - |vj.y - y_n||
    4: vj.x - x_n
    5: vj.y - y_n
    6: |vj.x - x_n| - |vj.y - y_n|
     */
    public Map<Obstacle, Map<PseudoBase, GurobiVariable[]>> aux_dInCorner_iqs;
    public Map<Obstacle, Map<PseudoBase, GurobiVariable>> auxQ_dInCorner;


    /*
    dOmOn_cqs
    0: d_m->n:MIN
    1: d_m->n:DIFF
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> dOmOn_cqs;



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
    Auxiliary absolute values: dist(v,v) without Detour
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
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vs_relObstaclesD_qs
    0: l->r
    1: r->l
    2: t->b
    3: b->t
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_relObstaclesD_qs;

    /*
    indicate relevant Obstacles_VS
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable>> vs_relObstacles_q;


    /*
    vs_detour_qs
    q_ij^d: detour trigger
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
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> vs_omOnCorner_qs;

    /*
    auxQuad_vs_OmOnCorner_qs
    0: cnnRules.3
    1: cnnRules.4
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> auxQuad_vs_omOnCorner_qs;


    /*
    vs_dOmOn_cqs
    0: vs_d_m->n:MIN
    1: vs_d_m->n:DIFF
     */
    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> vs_dOmOn_cqs;


    /*
    vs_inOutCnn_qs
    0: vi_s->
    1: ->sj
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_inOutCnn_qs;//binaryVar regarding Slaves for starting and end point connection

    /*
    vs_outCorner_qs
    0: vi->o.ll
    1: vi->o.ul
    2: vi->o.ur
    3: vi->o.lr
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_outCorner_qs;
    /*
    vs_inCorner_qs
    0: o.ll->sj
    1: o.ul->sj
    2: o.ur->sj
    3: o.lr->sj
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_inCorner_qs;
    /*
    auxQuad_vs_inOutCorner_qs
    0: cnnRules.5
    1: cnnRules.6
     */
    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> auxQuad_vs_inOutCorner_qs;

    /*
    vs_dOut_cqs: vi->o
    0: vs_odOut:MIN
    1: vs_odOut:DIFF
     */
    public Map<PseudoBase, GurobiVariable[]> vs_dOut_cqs;


    /*
    vs_dIn_cqs: o->pj
    0: vs_odIn:MIN
    1: vs_odIn:DIFF
     */
    public Map<PseudoBase, GurobiVariable[]> vs_dIn_cqs;

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
    public Map<PseudoBase, Map<Obstacle, Map<PseudoBase, GurobiVariable[]>>> aux_vs_dOutCorner_iqs;//first base is slave//second is corner
    public Map<PseudoBase, Map<Obstacle, Map<PseudoBase, GurobiVariable>>> auxQ_vs_dOutCorner;






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

        /*
        vp and vp->vp
         */
        this.non_qs = new HashMap<>();
        this.dir_qs = new HashMap<>();
        this.rel_qs = new HashMap<>();
        this.relD_qs = new HashMap<>();
        this.relObstacles_qs = new HashMap<>();
        this.relObstaclesD_qs = new HashMap<>();
        this.relObstacles_q = new HashMap<>();


        this.corner_qs = new HashMap<>();
        this.omOnCnn_q = new HashMap<>();
        this.omOnCorner_qs = new HashMap<>();
        this.auxQuad_omOnCorner_qs = new HashMap<>();
        this.inOutCnn_qs = new HashMap<>();
        this.outCorner_qs = new HashMap<>();
        this.inCorner_qs = new HashMap<>();
        this.auxQuad_inOutCorners_qs = new HashMap<>();

        this.dOmOn_cqs = new HashMap<>();

        this.aux_dOutCorner_iqs = new HashMap<>();
        this.auxQ_dOutCorner = new HashMap<>();
        this.aux_dInCorner_iqs = new HashMap<>();
        this.auxQ_dInCorner = new HashMap<>();


        this.vsCnn_q = new HashMap<>();


        /*
        vp->slave
         */
        this.vs_relObstacles_qs = new HashMap<>();
        this.vs_relObstaclesD_qs = new HashMap<>();
        this.vs_relObstacles_q = new HashMap<>();

        this.vs_detour_q = new HashMap<>();
        this.vs_corner_qs = new HashMap<>();

        this.vs_dist_cqs = new HashMap<>();

        this.vs_dOmOn_cqs = new HashMap<>();
        this.vs_omOnCnn_q = new HashMap<>();
        this.vs_omOnCorner_qs = new HashMap<>();
        this.auxQuad_vs_omOnCorner_qs = new HashMap<>();

        this.vs_inOutCnn_qs = new HashMap<>();
        this.vs_outCorner_qs = new HashMap<>();
        this.vs_inCorner_qs = new HashMap<>();
        this.vs_dOut_cqs = new HashMap<>();
        this.vs_dIn_cqs = new HashMap<>();
        this.auxQuad_vs_inOutCorner_qs = new HashMap<>();

        this.aux_vsDist_iqs = new HashMap<>();
        this.auxQ_vsDist = new HashMap<>();
        this.aux_vs_dOutCorner_iqs = new HashMap<>();
        this.auxQ_vs_dOutCorner = new HashMap<>();








        /*
        vp1->master
         */

        this.vm_relObstacles_qs = new HashMap<>();
        this.vm_relObstaclesD_qs = new HashMap<>();
        this.vm_relObstacles_q = new HashMap<>();

        this.vm_corner_qs = new HashMap<>();
        this.vm_omOnCnn_q = new HashMap<>();
        this.vm_omOnCorner_qs = new HashMap<>();
        this.auxQuad_vm_omOnCorner_qs = new HashMap<>();


        this.vm_inOutCnn_qs = new HashMap<>();
        this.vm_outCorner_qs = new HashMap<>();
        this.vm_inCorner_qs = new HashMap<>();
        this.auxQuad_vm_inOutCorner_qs = new HashMap<>();
        this.aux_vm_dOutCorner_iqs = new HashMap<>();
        this.auxQ_vm_dOutCorner = new HashMap<>();




        this.vm_dOmOn_cqs = new HashMap<>();









    }
}
