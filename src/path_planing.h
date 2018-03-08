#ifndef PATH_PLANING_H
#define PATH_PLANING_H

#include "fsm.h"
#include "json.hpp"

using namespace fsm;

namespace plan{
    struct Car{
        double x;
        double y;
        double s;
        float d;
        double yaw;
        double speed;

        void Fill(nlohmann::basic_json<>::value_type sensor_fusion, int prev_size){
            x = sensor_fusion[3];
            y = sensor_fusion[4];
            d = sensor_fusion[6];
            speed = sqrt(pow(x, 2) + pow(y,2));
            
            s = sensor_fusion[5];
            s += ((double) prev_size * 0.02 * speed);
        }
    };

    class PathPlaning{
        private:
            fsm::FSMachine machine;     //state machine
            Car car;                    //current car position

            int lane;                   //started lane
            double velocity;            //current velocity in mph
            int gap_to_change_lane;     //gap size needed to change lanes
            
            bool too_close;
			double lane_velocity;
            int wait_to_change_lane;    //time to change to the next 

        public:
            //constants
            static constexpr double MAX_VELOCITY = 49.5;    //reference velocity - mph
            static const int HORIZON = 50;                  //horizon - distance captured by sensors (in meters)
            static const int LANES = 3;                     //number of lanes

            //start functions
            PathPlaning(){
                this->lane = 1;
                this->gap_to_change_lane = 2 * HORIZON;
                this->velocity = 0;

                this->too_close = false;
                this->lane_velocity = PathPlaning::MAX_VELOCITY;
                this->wait_to_change_lane = 0;
            }

            void start(){
                if(machine.get() == finite_states::READY){
                    machine.next(); //start path planing (starts with READY)
                }
            }

            //getters and setters
            int getLane(){
                return this->lane;
            }

            int getGap(){
                return this->gap_to_change_lane;
            }

            double getVelocity(){
                return this->velocity;
            }

            Car getPosition(){
                return this->car;
            }

            void setPosition(Car car_data){
                this->car = car_data;
            }

            //helper functions
            double decreaseVelocity(){
                this->velocity -= 0.448;	//10 m/(s^2)
            }

            double increaseVelocity(){
                this->velocity += 0.448;	//10 m/(s^2)
            }

            bool isItInLane(float d, int ref_lane){
                return (d < (2+(4*ref_lane)+2)) && (d > (2+(4*ref_lane)-2));
            }

            bool isItInMyLane(float d){
                return isItInLane(d, (this->lane));
            }

            bool isItInMyLeftLane(float d){
                return isItInLane(d, (this->lane-1));
            }

            bool isItInMyRightLane(float d){
                return isItInLane(d, (this->lane+1));
            }

            bool moveToLeft(){
                if(lane > 0){
                    this->lane--;
                    return true;
                }
                return false;
            }

            bool moveToRight(){
                if(this->lane < (PathPlaning::LANES-1)){
                    this->lane++;
                    return true;
                }
                return false;
            }

            //control functions
            void checkLane(nlohmann::basic_json<>::value_type sensor_fusion, int prev_size){
                //if car is not doing any maneuver
                if(this->wait_to_change_lane <= 0)
                {
                    //find other cars using sensors
                    for(int i=0; i< sensor_fusion.size(); i++){
                        Car next_car;
                        next_car.Fill(sensor_fusion[i], prev_size);

                        //check car is in my current lane
                        if(this->isItInMyLane(next_car.d)){
                            //if the next car is in front of me
                            if((next_car.s > car.s) && ((next_car.s - car.s) < PathPlaning::HORIZON))
                            {
                                if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                                    this->lane_velocity = next_car.speed; 
                                }
                                else if(this->machine.get() == finite_states::LANE_KEEP){
                                    this->machine.next();
                                }
                            }
                        }
                        else if(this->isItInMyLeftLane(next_car.d)){
                            //if car is waiting for change lane, the change it
                            if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                                if((next_car.s > car.s) && ((next_car.s - car.s) < this->gap_to_change_lane)){  //if is enough space to change lanes
                                    if(this->moveToLeft()){ //can move to left
                                        this->machine.next(finite_states::LANE_CHANGE_LEFT);
                                        break;
                                    }
                                }
                            }
                            //else ignore car on the left
                        }
                        else if(this->isItInMyRightLane(next_car.d)){
                            if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                                if((next_car.s > car.s) && ((next_car.s - car.s) < this->gap_to_change_lane)){  //if is enough space to change lanes
                                    if(this->moveToRight()){ //can move to right
                                        this->machine.next(finite_states::LANE_CHANGE_RIGHT);
                                        break;
                                    }
                                }
                            }
                            //else ignore car on the left
                        }
                        
                        //decrease velocity
                        if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                            if(this->lane_velocity < car.speed){  //car is waiting for oportunity to change lane
                                this->decreaseVelocity();
                            }
                        }
                        else if(this->machine.get() == finite_states::LANE_KEEP){
                            this->lane_velocity = PathPlaning::MAX_VELOCITY;
                        }
                    }
                }
            }

            void doActions(){
                switch(this->machine.get()){
                    case finite_states::LANE_KEEP:
                        if(this->velocity < PathPlaning::MAX_VELOCITY){
                            this->increaseVelocity();
                        }

                        if(this->wait_to_change_lane > 0){
                            this->wait_to_change_lane--;
                        }
                        break;

                    case finite_states::LANE_CHANGE_LEFT:
                    case finite_states::LANE_CHANGE_RIGHT:
                        this->machine.next();
                        wait_to_change_lane = 0;//this->HORIZON;
                        break;

                    case finite_states::PREPARE_LANE_CHANGE:
                        this->decreaseVelocity();
                        break;
                }
            }
    };
}

#endif