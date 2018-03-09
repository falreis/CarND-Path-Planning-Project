#ifndef PATH_PLANING_H
#define PATH_PLANING_H

#include "fsm.h"
#include "json.hpp"
#include <cstdio>
#include <list>

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

    struct Move{
        bool toLeft;
        double distance;

        Move(){ }
        Move(bool isleft, double dist){
            this->toLeft = isleft;
            this->distance = dist;
        }
    };

    class PathPlaning{
        private:
            fsm::FSMachine machine;     //state machine
            Car car;                    //current car position
            list<Move> possible_moves;  //possible lane moves

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
                possible_moves.clear();
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
                        double distance = (next_car.s - car.s);

                        //check car is in my current lane
                        if(this->isItInMyLane(next_car.d)){
                            //if the next car is in front of me
                            if(distance > 0 && distance < PathPlaning::HORIZON)
                            {
                                if(this->machine.get() == finite_states::LANE_KEEP 
                                    || this->machine.get() == finite_states::PREPARE_LANE_CHANGE
                                ){
                                    this->machine.next();
                                    this->lane_velocity = next_car.speed;
                                    break;
                                }
                            }
                        }
                    }

                    //verify if needs to decrease velocity
                    if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                        if(this->lane_velocity < this->velocity){  //car is waiting for oportunity to change lane
                            this->decreaseVelocity();
                        }
                        else{
                            this->increaseVelocity();
                        }
                    }
                    else if(this->machine.get() == finite_states::LANE_KEEP){
                        this->lane_velocity = PathPlaning::MAX_VELOCITY;
                    }
                    
                    //find possible moves if there is a car in front of my car
                    if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                        for(int i=0; i< sensor_fusion.size(); i++){
                            Car next_car;
                            next_car.Fill(sensor_fusion[i], prev_size);
                            double distance = (next_car.s - car.s);

                            if(this->isItInMyLeftLane(next_car.d)){
                                //if car is waiting for change lane, the change it
                                if(abs(distance) < this->gap_to_change_lane){  //if is not enough space to change lanes
                                    possible_moves.push_back(Move(true, distance));
                                }
                            }
                            else if(this->isItInMyRightLane(next_car.d)){
                                if(abs(distance) < this->gap_to_change_lane){  //if is not enough space to change lanes
                                    possible_moves.push_back(Move(false, distance));
                                }
                            }
                        }
                    }

                    //if it's possible some maneuver, prefer left than right
                    if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE && possible_moves.size() == 0){ //no problems to move
                        this->machine.next(finite_states::LANE_CHANGE_LEFT);
                    }
                    else if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE && possible_moves.size() > 0){
                        bool can_move_left = true;
                        bool can_move_right = true;

                        for (list<Move>::iterator it= possible_moves.begin(); it != possible_moves.end(); ++it){
                            if(it->toLeft) 
                                can_move_left = false;
                            else
                                can_move_right = false;
                        }

                        //verify if can move left or right then proceed move
                        if(can_move_left){
                            if(this->moveToLeft()){     //possible to move left
                                this->machine.next(finite_states::LANE_CHANGE_LEFT);
                            }
                        }
                        else if(can_move_right){
                            if(this->moveToRight()){     //possible to move right
                                this->machine.next(finite_states::LANE_CHANGE_RIGHT);
                            }
                        }
                        //else just decrease velocity

                        possible_moves.clear(); //clean possible maneuvers
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
                        this->machine.next();       //back to lane keep
                        wait_to_change_lane = 30;   //this->HORIZON;
                        break;

                    case finite_states::PREPARE_LANE_CHANGE:
                        this->decreaseVelocity();
                        break;
                }
            }
    };
}

#endif