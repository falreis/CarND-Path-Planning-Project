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
        double speed;       //in mps
        double velocity;    //in mph

        void Fill(nlohmann::basic_json<>::value_type sensor_fusion, int prev_size){
            this->x = sensor_fusion[3];
            this->y = sensor_fusion[4];
            this->d = sensor_fusion[6];
            this->speed = sqrt(pow(x, 2) + pow(y,2));
            this->velocity = this->speed * 2.23;
            
            this->s = sensor_fusion[5];
            this->s += ((double) prev_size * 0.02 * this->speed);
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
                this->gap_to_change_lane = 0.5 * HORIZON;   //gap is in both directions - front and back
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

            bool canMoveLeft(){
                return (this->lane > 0);
            }

            bool canMoveRight(){
                return (this->lane < (PathPlaning::LANES-1));
            }

            bool moveToLeft(){
                if(this->canMoveLeft()){
                    this->lane--;
                    return true;
                }
                return false;
            }

            bool moveToRight(){
                if(this->canMoveRight()){
                    this->lane++;
                    return true;
                }
                return false;
            }

            //control functions
            void checkLane(nlohmann::basic_json<>::value_type sensor_fusion, int prev_size){
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
                            //if my car it's too close from the front car, decrease velocity
                            if(distance < this->gap_to_change_lane){
                                this->decreaseVelocity();
                            }

                            //prepare to change lane
                            if(this->machine.get() == finite_states::LANE_KEEP){
                                this->machine.next();
                                this->lane_velocity = next_car.velocity;
                                break;
                            }
                            else if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                                this->lane_velocity = next_car.velocity;
                                break;
                            }
                        }
                    }
                }
                
                //find possible moves if there is a car in front of my car
                if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                    for(int i=0; i< sensor_fusion.size(); i++){
                        Car next_car;
                        next_car.Fill(sensor_fusion[i], prev_size);
                        double distance = (next_car.s - car.s);

                        if(this->isItInMyLeftLane(next_car.d)){
                            if(this->canMoveLeft()){
                                if(abs(distance) < this->gap_to_change_lane){  //if is not enough space to change lanes
                                    possible_moves.push_back(Move(true, distance));
                                }
                            }
                        }
                        else if(this->isItInMyRightLane(next_car.d)){
                            if(this->canMoveRight()){
                                if(abs(distance) < this->gap_to_change_lane){  //if is not enough space to change lanes
                                    possible_moves.push_back(Move(false, distance));
                                }
                            }
                        }
                    }
                }

                //if it's possible some maneuver, prefer left than right
                if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE && possible_moves.size() == 0){ //no problems to move
                    if(this->moveToLeft()){
                        this->machine.next(finite_states::LANE_CHANGE_LEFT);
                    }
                }
                else if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE && possible_moves.size() > 0){
                    bool go_left = true;
                    bool go_right = true;

                    for (list<Move>::iterator it= possible_moves.begin(); it != possible_moves.end(); ++it){
                        if(it->toLeft) 
                            go_left = false;
                        else
                            go_right = false;
                    }

                    //verify if can move left or right then proceed move
                    if(go_left){
                        if(this->moveToLeft()){     //possible to move left
                            this->machine.next(finite_states::LANE_CHANGE_LEFT); 
                        }
                    }
                    else if(go_right){
                        if(this->moveToRight()){     //possible to move right
                            this->machine.next(finite_states::LANE_CHANGE_RIGHT);
                        }
                    }
                    //else just decrease velocity

                    possible_moves.clear(); //clean possible maneuvers
                }
            }

            void doActions(){
                switch(this->machine.get()){
                    case finite_states::LANE_KEEP:
                        this->lane_velocity = PathPlaning::MAX_VELOCITY;
                        if(this->velocity < PathPlaning::MAX_VELOCITY){
                            this->increaseVelocity();
                        }
                        break;

                    case finite_states::LANE_CHANGE_LEFT:
                    case finite_states::LANE_CHANGE_RIGHT:
                        this->machine.next();       //back to lane keep
                        break;

                    case finite_states::PREPARE_LANE_CHANGE:
                        if(this->lane_velocity < this->velocity){  //car is waiting for oportunity to change lane
                            this->decreaseVelocity();
                        }
                        else{
                            this->increaseVelocity();
                        }
                        break;
                }
            }
    };
}

#endif