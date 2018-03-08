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

    class FSMachine{
        private:
            finite_states curr_state;
        public:
            FSMachine(){
                this->curr_state = finite_states::READY;
            }

            finite_states Get(){
                return this->curr_state;
            }

            bool Next(){
                switch(this->curr_state){
                    case finite_states::READY:
                        this->curr_state = finite_states::LANE_KEEP;

                    case finite_states::PREPARE_LANE_CHANGE_LEFT:
                        this->curr_state = finite_states::LANE_CHANGE_LEFT;

                    case finite_states::PREPARE_LANE_CHANGE_RIGHT:
                        this->curr_state = finite_states::LANE_CHANGE_RIGHT;

                    case finite_states::LANE_CHANGE_LEFT:
                    case finite_states::LANE_CHANGE_RIGHT:
                        this->curr_state = finite_states::LANE_KEEP;

                    default:
                        return false;   //invalid state
                }
                return true;
            }

            bool Next(finite_states next_state){
                switch(this->curr_state){
                    case finite_states::LANE_KEEP:
                        if(next_state == finite_states::PREPARE_LANE_CHANGE_LEFT 
                            || next_state == finite_states::PREPARE_LANE_CHANGE_RIGHT
                        ){
                            this->curr_state = next_state;
                        }
                    default:
                        return false;   //invalid state
                }
                return true;
            }

            bool Previous(){
                switch(this->curr_state){
                    case finite_states::PREPARE_LANE_CHANGE_LEFT:
                    case finite_states::PREPARE_LANE_CHANGE_RIGHT:
                        this->curr_state = finite_states::LANE_KEEP;

                    default:
                        return false;   //invalid state
                }
                return true;
            }
    };
}

#endif