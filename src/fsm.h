#ifndef FSM_H
#define FSM_H

//finite state machine

namespace fsm{
    enum finite_states{
        READY = 0,
        LANE_KEEP = 1,
        PREPARE_LANE_CHANGE_LEFT = 2,
        LANE_CHANGE_LEFT = 3,
        PREPARE_LANE_CHANGE_RIGHT = 4,
        LANE_CHANGE_RIGHT = 5
    };
}

#endif