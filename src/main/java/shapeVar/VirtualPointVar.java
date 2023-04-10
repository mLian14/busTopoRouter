package shapeVar;

import grb.GurobiVariable;
import shapes.Obstacle;

import java.util.ArrayList;
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
    Detour triggering
     */

    /*
    o_vp_Non_qs
    0: nonL
    1: nonR
    2: nonA
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
    4: relative obstacle: aux.2
     */
    public Map<Obstacle, GurobiVariable[]> o_vp_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vp_detour_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
    4: q_ij^d: detour trigger: aux.3
     */
    public GurobiVariable[] vvDetour_qs;//binary variables regarding next virtualPoint for (ul->lr.2) -- (ll->ur.2)



    /*
    Path Length Computation
     */

    /*
    o_vvCorner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<Obstacle, GurobiVariable[]> o_vvCorner_qs;

    /*
    vv_ooCnn_qs
    0: q_ij^m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vv_ooCnn_qs;//binaryVar regarding next virtualPoint for relObstacles' connection

    /*
    vv_inOutCnn_qs
    0: ->
    1: <-
     */
    public Map<Obstacle, GurobiVariable[]> vv_inOutCnn_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    /*
    vv_oCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> vv_oCoordinate_iqs;//intVar regarding next virtualPoint: coordinates of the selected intermedia point

    /*
    vv_ooDist_iqs
    0: d_m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vv_ooDist_iqs;//intVar: path length between o_m and o_n

    /*
    vv_inOutDist_iqs
    0: d_->
    1: d_<-
     */
    public GurobiVariable[] vv_inOutDist_iqs;//intVar

    /*
    dist_iqs
    0: vv dist
    1: v.corrS dist
    2: vm dist (only for 1st vp)
     */
    public GurobiVariable[] dist_iqs;


    public VirtualPointVar() {

        this.o_vp_Non_qs = new HashMap<>();
        this.o_vp_Rel_qs = new HashMap<>();
        this.o_vp_diagonalSets_qs = new HashMap<>();
        this.o_vp_relObstacles_qs = new HashMap<>();


        this.o_vvCorner_qs = new HashMap<>();
        this.vv_ooCnn_qs = new HashMap<>();
        this.vv_inOutCnn_qs = new HashMap<>();
        this.vv_oCoordinate_iqs = new HashMap<>();
        this.vv_ooDist_iqs = new HashMap<>();

    }
}
