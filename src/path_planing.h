#ifndef PATH_PLANING_H
#define PATH_PLANING_H

#include "fsm.h"

namespace plan{
    class PathPlaning{
        private:
            fsm::FSMachine machine;         //state machine
            int lane;                       //started lane
            double velocity;                //current velocity in mph
            int gap_to_change_lane;         //gap size needed to change lanes

        public:
            //constants
            static constexpr double MAX_VELOCITY = 49.5;    //reference velocity - mph
            static const int HORIZON = 50;                  //horizon - distance captured by sensors (in meters)

            //functions
            PathPlaning(){
                this->lane = 1;
                this->gap_to_change_lane = 2 * HORIZON;
                this->velocity = 0;
            }

            int getLane(){
                return this->lane;
            }

            int getGap(){
                return this->gap_to_change_lane;
            }

            double getVelocity(){
                return this->velocity;
            }

            double decreaseVelocity(){
                this->velocity -= 0.448;	//10 m/(s^2)
            }

            double increaseVelocity(){
                this->velocity += 0.448;	//10 m/(s^2)
            }

            void start(){
                if(machine.Get() == fsm::finite_states::READY)
                    machine.Next(); //start path planing (starts with READY)
            }

            bool isItInMyLane(float d){
                return (d < (2+(4*this->lane)+2)) && (d > (2+(4*this->lane)-2));
            }
    };
}

#endif