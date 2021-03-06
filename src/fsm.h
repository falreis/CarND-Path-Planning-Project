#ifndef FSM_H
#define FSM_H

//finite state machine

#include <iostream>

namespace fsm{
    enum finite_states{
        READY = 0,
        LANE_KEEP = 1,
        //PREPARE_LANE_CHANGE_LEFT = 2,
        LANE_CHANGE_LEFT = 3,
        //PREPARE_LANE_CHANGE_RIGHT = 4,
        LANE_CHANGE_RIGHT = 5,
        PREPARE_LANE_CHANGE = 6
    };

    class FSMachine{
        private:
            finite_states curr_state;
        public:
            FSMachine(){
                this->curr_state = finite_states::READY;
                this->printState();
            }

            finite_states get(){
                return this->curr_state;
            }

            bool next(){
                switch(this->curr_state){
                    case finite_states::READY:
                        this->curr_state = finite_states::LANE_KEEP;
                        break;

                    /*
                    case finite_states::PREPARE_LANE_CHANGE_LEFT:
                        this->curr_state = finite_states::LANE_CHANGE_LEFT;
                        break;

                    case finite_states::PREPARE_LANE_CHANGE_RIGHT:
                        this->curr_state = finite_states::LANE_CHANGE_RIGHT;
                        break;
                    */
                    case finite_states::LANE_KEEP:
                        this->curr_state = finite_states::PREPARE_LANE_CHANGE;
                        break;

                    case finite_states::LANE_CHANGE_LEFT:
                    case finite_states::LANE_CHANGE_RIGHT:
                        this->curr_state = finite_states::LANE_KEEP;
                        break;

                    default:
                        return false;   //invalid state
                }

                this->printState();
                return true;
            }

            bool next(finite_states next_state){
                switch(this->curr_state){
                    /*case finite_states::LANE_KEEP:
                        if(next_state == finite_states::PREPARE_LANE_CHANGE_LEFT 
                            || next_state == finite_states::PREPARE_LANE_CHANGE_RIGHT
                        ){
                            this->curr_state = next_state;
                        }
                    */
                    case finite_states::PREPARE_LANE_CHANGE:
                        if(next_state == finite_states::LANE_CHANGE_LEFT || next_state == finite_states::LANE_CHANGE_RIGHT){
                            this->curr_state = next_state;
                        }
                        break;

                    default:
                        return false;   //invalid state
                }

                this->printState();
                return true;
            }

            bool previous(){
                switch(this->curr_state){
                    /*case finite_states::PREPARE_LANE_CHANGE_LEFT:
                    case finite_states::PREPARE_LANE_CHANGE_RIGHT:*/
                    case finite_states::PREPARE_LANE_CHANGE:
                        this->curr_state = finite_states::LANE_KEEP;
                        break;

                    default:
                        return false;   //invalid state
                }

                this->printState();
                return true;
            }

            void printState(){
                string state_name;

                switch(this->curr_state){
                    case finite_states::READY:
                        state_name = "READY";
                        break;

                    case finite_states::LANE_KEEP:
                        state_name = "LANE_KEEP";
                        break;

                    /*case finite_states::PREPARE_LANE_CHANGE_LEFT:
                        state_name = "PREPARE_LANE_CHANGE_LEFT";
                        break;*/

                    case finite_states::LANE_CHANGE_LEFT:
                        state_name = "LANE_CHANGE_LEFT";
                        break;

                    /*case finite_states::PREPARE_LANE_CHANGE_RIGHT:
                        return "PREPARE_LANE_CHANGE_RIGHT";
                        break;*/
                    case finite_states::LANE_CHANGE_RIGHT:
                        state_name = "LANE_CHANGE_RIGHT";
                        break;

                    case finite_states::PREPARE_LANE_CHANGE:
                        state_name = "PREPARE_LANE_CHANGE";
                        break;
                }
                
                std::cout<<state_name<<std::endl;
            }
    };
}

#endif