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
            //this->s += ((double) prev_size * 0.02 * this->speed);
        }
    };

    enum direction{
        KEEP = 1,
        LEFT = 2,
        RIGHT = 3
    };

    struct Move{
        direction moveDirection;
        double distance;
        double velocity;

        Move(){ }
        Move(direction dir, double dist, double vel){
            this->moveDirection = dir;
            this->distance = dist;
            this->velocity = vel;
        }
    };

    class PathPlaning{
        private:
            fsm::FSMachine machine;     //state machine
            Car car;                    //current car position
            list<Move> unsafe_moves;    //possible lane moves
            list<Move> front_cars;      //cars in front of my car

            int lane;                   //started lane
            double velocity;            //current velocity in mph
            
			double lane_velocity;       //max lane velocity (for traffic circunstances)
            double wait_time_maneuver;  //waiting time until finish maneuver
            double gap_to_change_lane;  //gap size needed to change lanes
            double front_car_distance;  //front car distance
        
        protected:
            /* control functions */
            void currentLaneControl(nlohmann::basic_json<>::value_type sensor_fusion, int prev_size){
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
                            this->front_cars.push_back(Move(direction::KEEP, distance, next_car.velocity));

                            //prepare to change lane
                            if(this->machine.get() == finite_states::LANE_KEEP){
                                this->machine.next();
                            }
                        }
                    }
                }

                //keep velocity as the same for the next front car
                Move front_car;
                front_car.distance = 9999999999;
                
                if(front_cars.size() > 0){
                    for(list<Move>::iterator it=front_cars.begin(); it != front_cars.end(); ++it){
                        if(it->distance < front_car.distance){
                            front_car = *it;
                        }
                    }

                    //if my car it's too close from the front car, decrease velocity
                    if(front_car.distance < PathPlaning::HORIZON){
                        //this->decreaseVelocity();
                        this->lane_velocity = front_car.velocity;
                        this->front_car_distance = abs(front_car.distance);
                    }
                }
                else{
                    this->lane_velocity = PathPlaning::MAX_VELOCITY;
                }
                front_cars.clear();
            }

            void changeLaneControl(nlohmann::basic_json<>::value_type sensor_fusion, int prev_size){
                //find possible moves if there is a car in front of my car
                if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE){
                    for(int i=0; i< sensor_fusion.size(); i++){
                        Car next_car;
                        next_car.Fill(sensor_fusion[i], prev_size);
                        double distance = (next_car.s - car.s);
                            
                        //verify if sensors detect cars on left or right lanes
                        if(this->isItInMyLeftLane(next_car.d)){
                            if(this->canMoveLeft()){
                                if(!this->isChangeWorth(distance)){    //if is not enough space to change lanes
                                    cout<<"Left: "<<abs(distance)<<" - "<<PathPlaning::DIST_CHANGE_LANE<< std::endl;
                                    unsafe_moves.push_back(Move(direction::LEFT, distance, next_car.velocity));
                                }
                            }
                        }
                        else if(this->isItInMyRightLane(next_car.d)){
                            if(this->canMoveRight()){
                                if(!this->isChangeWorth(distance)){  //if is not enough space to change lanes
                                    cout<<"Right: "<<abs(distance)<<" - "<<PathPlaning::DIST_CHANGE_LANE<< std::endl;
                                    unsafe_moves.push_back(Move(direction::RIGHT, distance, next_car.velocity));
                                }
                            }
                        }
                    }
                }

                //if it's possible some maneuver, prefer left than right
                if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE && unsafe_moves.size() == 0){ //no problems to move
                    if(this->moveToLeft()){
                        this->machine.next(finite_states::LANE_CHANGE_LEFT);
                    }
                    else if(this->moveToRight()){
                        this->machine.next(finite_states::LANE_CHANGE_RIGHT);
                    }
                }
                else if(this->machine.get() == finite_states::PREPARE_LANE_CHANGE && unsafe_moves.size() > 0){
                    bool go_left = true;
                    bool go_right = true;

                    for (list<Move>::iterator it= unsafe_moves.begin(); it != unsafe_moves.end(); ++it){
                        if(it->moveDirection == direction::LEFT) 
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
                }

                unsafe_moves.clear(); //clean possible maneuvers
            }

        public:
            /* constants */
            static constexpr double MAX_VELOCITY = 49.5;    //reference velocity - mph
            static const int HORIZON = 50;                  //horizon - distance captured by sensors (in meters)
            static const int LANES = 3;                     //number of lanes
            static const int DIST_CHANGE_LANE = 10;         //gap size needed to change lanes

            /* start functions*/

            PathPlaning(){
                this->lane = 1;
                this->velocity = 45;
                this->lane_velocity = PathPlaning::MAX_VELOCITY;
                this->wait_time_maneuver = 0;
                this->front_car_distance = 0;

                this->unsafe_moves.clear();
                this->front_cars.clear();
            }

            void start(){
                if(machine.get() == finite_states::READY){
                    machine.next(); //start path planing (starts with READY)
                }
            }

            /* getters and setters */

            int getLane(){
                return this->lane;
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

            /* helper functions */
            double decreaseVelocity(){
                this->velocity -= 0.448;	//10 m/(s^2)
            }

            double increaseVelocity(){
                this->velocity += 0.448;	//10 m/(s^2)
            }

            bool amIInLane(){
                float pos = (this->car.d / (this->lane*4.0));
                return (pos > 0.3 && pos < 0.7);
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

            bool isChangeWorth(int distance){
                //TODO: verificar se os sensores não estão retornando carros em outra direção
                if(this->front_car_distance > 0){
                    if(distance < 0)
                        return abs(distance) > PathPlaning::DIST_CHANGE_LANE;
                    else
                        return (abs(distance) > PathPlaning::DIST_CHANGE_LANE && distance > this->front_car_distance);
                }
                return true;
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

            /* control functions */
            void checkLane(nlohmann::basic_json<>::value_type sensor_fusion, int prev_size){
                //control the car in current lane (define best velocity, decide to change lane)
                this->currentLaneControl(sensor_fusion, prev_size);

                //find possible moves (change lane)
                if(this->wait_time_maneuver <= 0){
                //if(this->amIInLane()){
                    this->changeLaneControl(sensor_fusion, prev_size);
                }
                else{
                    this->wait_time_maneuver--;  
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
                        this->wait_time_maneuver = PathPlaning::HORIZON;
                        this->front_car_distance = 0;
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