/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Giulio Mazzi (2017-2018)
*********************************************************************/

#include "SSIPatrolAgent.h"
#include <std_srvs/Empty.h>

//Sequential Single Item Auction with dynamic compact partition of the environment
class DTASSILearning_Agent: public SSIPatrolAgent {

    protected:
        //center location given current tasks: task location that is at minimum path distance from all other locations. This is always a task location.
        size_t current_center_location;

        //compute minimum path cost considering all tasks (tasks) and the next vertex (nv).
        //The first room is always the current goal (if any), then rooms are visited in decreasing order of utility.
        //The path cost is sum of travel cost given the order.
        double compute_bid(int nv);

        //update tasks setting to true only the vertices for which this robot has the current highest bid
        //based on the array bids
        void update_tasks();

        //compute center point given current tasks
        void compute_center_location();

        double compute_sum_distance(int cv);

        //float** learnign_weigh;

        // time and space at witch the robot start from the last node
        double extimation_starting_time;
        double travel_time_prediction;
        uint   extimation_starting_point;

        // filter new weight
        double alpha;

        // path to goal, required to evaluate every edge of the graph by itself
        int *path_to_goal;
        uint to_goal_size;
        uint to_goal_position;

        // rember waiting time
        double real_goal_reached_wait;

        void learn_from_edge();
        void make_prediction();
    public:

        DTASSILearning_Agent(){}

        virtual void run();
        void onGoalComplete();
        void onGoalNotComplete();
        double compute_cost(int start, int end);
        void init(int argc, char** argv);
};

void DTASSILearning_Agent::init(int argc, char** argv) {

    //    logfile = fopen("DTASSIOut.log","w");

    //    fprintf(logfile,"INITIALIZING \n");
    //    fflush(logfile);

    SSIPatrolAgent::init(argc,argv);

    //    fprintf(logfile,"INITIALIZING 2 \n");
    //    fflush(logfile);

    //set current center location to current vertex
    current_center_location = current_vertex;

    //    fprintf(logfile,"initialised current center location to: %d \n",current_center_location);

    //initialize parameters

    extimation_starting_point = current_vertex;
    alpha = 0.0;

    path_to_goal = new int[dimension];
    to_goal_size = 0;
    to_goal_position = 0;

    // rember waiting time
    real_goal_reached_wait = goal_reached_wait;

    if ( ID_ROBOT == 0 || ID_ROBOT == 1 ) {
        //set a slower speed
        for ( int i =0; i < dimension; ++i) {
            for ( int j = 0; j < vertex_web[i].num_neigh ; ++j)
                vertex_web[i].cost[j] = vertex_web[i].cost[j] * 2;
        }
    }

}

void DTASSILearning_Agent::onGoalComplete()
{

    //printf("DTAPL onGoalComplete!!!\n");
    if (first_vertex){

        next_vertex = compute_next_vertex(current_vertex);
        first_vertex = false;

        // calculate path to goal
        dijkstra(current_vertex, next_vertex, path_to_goal, to_goal_size, vertex_web, dimension);
        to_goal_position = 0;

        /** SEND GOAL (REACHED) AND INTENTION **/
        send_goal_reached(); // Send TARGET to monitor
        send_results();  // Algorithm specific function

    } else {
        // we are on then next vertices
        ++to_goal_position;
        printf("reached %d vertex\n",path_to_goal[to_goal_position]);

        // learning
        learn_from_edge();

        // we are in the real goal?
        if ( to_goal_position == (to_goal_size-1) ) {
            printf("reached goal\n");
            //Update Idleness Table:
            update_global_idleness();
            //update current vertex
            current_vertex = next_vertex;
            //update next vertex based on previous decision
            next_vertex = next_next_vertex;
            //reset next_next
            next_next_vertex = -1;

            //update global idleness of next vertex to avoid conflicts
            if (next_vertex>=0 && next_vertex< dimension){
                pthread_mutex_lock(&lock);
                global_instantaneous_idleness[next_vertex] = 0.0;
                pthread_mutex_unlock(&lock);
            }

            // calculate path to goal
            dijkstra(current_vertex, next_vertex, path_to_goal, to_goal_size, vertex_web, dimension);
            to_goal_position = 0;

            /** SEND GOAL (REACHED) AND INTENTION **/
            send_goal_reached(); // Send TARGET to monitor
            send_results();  // Algorithm specific function
        }
    }

    // set a proper waiting time
    if ( to_goal_position == to_goal_size-2 )
        goal_reached_wait = real_goal_reached_wait;
    else
        goal_reached_wait = 0.0;

    // prediction for learning
    make_prediction();

    //Send the goal to the robot (Global Map)
    sendGoal(path_to_goal[to_goal_position+1]);  // send to move_base
    goal_complete = false;

    //compute next next vertex
    if ( next_next_vertex == -1 )
        next_next_vertex = compute_next_vertex(next_vertex);

    printf("<<< DONE Computed next vertices: current_vertex = %d, next_vertex=%d, next_next_vertex=%d >>>\n",current_vertex, next_vertex,next_next_vertex);

}

void DTASSILearning_Agent::learn_from_edge() {

    uint extimation_starting_point = path_to_goal[to_goal_position-1];
    uint actual_vertex = path_to_goal[to_goal_position];

    // learn from prediction
    double real_travel_time = ros::Time::now().toSec() - extimation_starting_time;

    // find neighbourn positions
    int id_neigh = is_neigh(extimation_starting_point, actual_vertex, vertex_web, dimension);
    int id_neigh2 = is_neigh(actual_vertex, extimation_starting_point, vertex_web, dimension);

    // calculate new weight
    double old_weight = vertex_web[extimation_starting_point].cost[id_neigh];
    double travel_ratio = (real_travel_time / travel_time_prediction);

    // fix a maximum bound for problematic travel
    if ( travel_ratio < 0.5 ) travel_ratio = 0.5;
    if ( travel_ratio > 2.0 ) travel_ratio = 2.0;

    double new_weight = old_weight * travel_ratio;

    // update value
    vertex_web[extimation_starting_point].cost[id_neigh] = alpha * ( new_weight ) + (1.0-alpha) * old_weight;
    vertex_web[actual_vertex].cost[id_neigh2] = alpha * ( new_weight ) + (1.0-alpha) * old_weight;

    printf("traveled time from %d to %d:\n"
            "real time      : %fs\n"
            "predicted time : %fs\n"
            "old weight : %f\n"
            "new weight : %f\n",
            extimation_starting_point, actual_vertex,real_travel_time,
            travel_time_prediction, old_weight, vertex_web[extimation_starting_point].cost[id_neigh]);
}

void DTASSILearning_Agent::make_prediction() {
    // prediction on distance
    extimation_starting_time = ros::Time::now().toSec();
    //travel_time_prediction = learnign_weigh[extimation_starting_point][next_vertex]*(compute_cost(extimation_starting_point,next_vertex));
    travel_time_prediction = compute_cost(path_to_goal[to_goal_position],
            path_to_goal[to_goal_position+1]);

    // prediction on rotation
    float y_diff = vertex_web[next_vertex].y - vertex_web[current_vertex].y;
    float x_diff = vertex_web[next_vertex].x - vertex_web[current_vertex].x;
    const float PI = 3.1415;

    // yaw to new point
    float  pred_yaw;
    if ( std::abs(x_diff) > 0.00001f ) {
        pred_yaw = atan( y_diff / x_diff);
        if ( x_diff < 0.0f ) {
            if ( y_diff < 0.0f )
                pred_yaw-=PI;
            else
                pred_yaw+=PI;
        }
    }
    else {
        if ( y_diff > 0.0f )
            pred_yaw = PI/2.0f;
        else
            pred_yaw = -(PI/2.0f);
    }

    //printf("NEXT ANGLE %f", pred_yaw*(180.0f/PI));
    float pose_x, pose_y, current_yaw;
    getRobotPose(ID_ROBOT,pose_x,pose_y,current_yaw);
    //printf("  CURRENT ANGLE: %f\n", current_yaw*(180.0f/PI));
    float rotate_prediction = std::min(std::abs(pred_yaw-current_yaw),std::abs(-PI - current_yaw)+(PI - pred_yaw));

    travel_time_prediction += rotate_prediction;
}

double DTASSILearning_Agent::compute_bid(int nv){

    // is next vertex usefull?
    if (nv==next_vertex || nv==next_next_vertex) return 0.;

    // find the number of required task
    size_t num_tasks = 1;
    for (size_t i = 0; i<dimension ; i++) if (tasks[i]) num_tasks++;

    // compute the cost
    double bid_value =
        compute_cost(nv,current_center_location)*
        num_tasks;
        //( 1.0 / learnign_weigh[nv][current_center_location]);

    //printf("\nBID VALUE %f for %d, current %d, center %ld\n\n",bid_value, nv, current_vertex, current_center_location);

    return bid_value;
}

void DTASSILearning_Agent::compute_center_location(){
    size_t min = current_vertex;
    //    printf("compute center:: min: %d current center: %d \n",min,current_center_location);
    double min_dist = compute_sum_distance(min);
    //    printf("compute center:: min dist: %.2f \n",min_dist);
    for (size_t i = 0; i<dimension; i++){
        if (i!=current_center_location && tasks[i]){
            double dist = compute_sum_distance(i);
            //          printf("compute center:: current dist: %.2f, min dist: %.2f, current min: %d, current point: %d \n",dist,min_dist,min,i);
            if ( dist < min_dist){
                min = i;
                min_dist = dist;
            }
        }
    }
    current_center_location = min;

}

double DTASSILearning_Agent::compute_sum_distance(int cv){
    if(cv<0 || cv >= dimension){
        //          printf("return big number: cv = %d",cv);
        return BIG_NUMBER;
    }
    double sum = 0.;
    for (size_t i = 0; i<dimension ; i++){
        if (tasks[i]){
            //          printf("sum: %2.f \n",sum);
            sum+= compute_cost(cv,i);
        }
    }
    return sum;
}

void DTASSILearning_Agent::update_tasks() {

    /*debug print
      printf("updating tasks: \n tasks before[");
      for (size_t i = 0; i<dimension; i++){
      printf(" %d, ",tasks[i]);
      }
      printf("] \n");

      printf("bids [");
      for (size_t i = 0; i<dimension; i++){
      printf(" <%.2f,%d>, ",bids[i].bidValue,bids[i].robotId);
      }
      printf("] \n");

      printf("center location before %d \n",current_center_location);

      ------------*/

    int value = ID_ROBOT;
    if (value==-1){value=0;}

    nactivetasks=0;
    bool changed = false;
    for (size_t i = 0; i< dimension; i++){
        if (!changed && tasks[i] != (bids[i].robotId == value)){
            changed = true;
        }
        tasks[i] = (bids[i].robotId == value);
        if (tasks[i]) nactivetasks++;
    }

    if (changed){
        compute_center_location();
    }

#if DEBUG_PRINT

    printf("DTAPL current center location: %lu\n",current_center_location);
    printf("DTAPL: Active Tasks %d [",nactivetasks);
    for (size_t i = 0; i<dimension; i++){
        if (tasks[i]) printf("%lu ",i);
    }
    printf("] \n");

#endif
}

double DTASSILearning_Agent::compute_cost(int cv, int nv)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension];
    int id_neigh;

    dijkstra(cv, nv, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
    double distance = 0;

    for(uint j=0; j<elem_s_path; j++){
//        printf("path[%u] = %d\n",j,shortest_path[j]);
        if (j<elem_s_path-1){
            id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
            distance += vertex_web[shortest_path[j]].cost[id_neigh];
        }
    }

    return distance;
}

/* modified version of run, needed to account the different handgling
 * of path planning
 */
void DTASSILearning_Agent::run() {
    
    // get ready
    ready();

    //initially clear the costmap (to make sure the robot is not trapped):
    std_srvs::Empty srv;
    std::string mb_string;

    if (ID_ROBOT>-1){
        std::ostringstream id_string;
        id_string << ID_ROBOT;
        mb_string = "robot_" + id_string.str() + "/";
    }
    mb_string += "move_base/clear_costmaps";

    if (ros::service::call(mb_string.c_str(), srv)){
        ROS_INFO("Costmap correctly cleared before patrolling task.");
    }else{
        ROS_WARN("Was not able to clear costmap (%s) before patrolling...",
                mb_string.c_str());
    }

    // Asynch spinner (non-blocking)
    ros::AsyncSpinner spinner(2); // Use n threads
    spinner.start();

    /* Run Algorithm */ 
    
    ros::Rate loop_rate(30); //0.033 seconds or 30Hz
    
    while(ros::ok()){

        if (goal_complete) {
            onGoalComplete();  // can be redefined
            resend_goal_count=0;
        }
        else { // goal not complete (active)
            if (interference) {
                do_interference_behavior();
            }       

            if (ResendGoal) {
                //Send the goal to the robot (Global Map)
                if (resend_goal_count<3) {
                    resend_goal_count++;
                    ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)",
                            resend_goal_count, path_to_goal[to_goal_position+1],
                            vertex_web[path_to_goal[to_goal_position+1]].x,
                            vertex_web[path_to_goal[to_goal_position+1]].y);

                    sendGoal(path_to_goal[to_goal_position+1]);
                }
                else {
                    resend_goal_count=0;
                    onGoalNotComplete();
                }
                ResendGoal = false;
            }

            processEvents();

            if (end_simulation) return;
        }
        loop_rate.sleep(); 
    }
}

void DTASSILearning_Agent::onGoalNotComplete()
{   
    int prev_vertex = next_vertex;
    
    ROS_INFO("Goal not complete - From vertex %d to vertex %d\n", current_vertex, next_vertex);   
    
    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex = compute_next_vertex();
    //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    // Look for a random adjacent vertex different from the previous one
    int random_cnt=0;
    while (next_vertex == prev_vertex && random_cnt++<10) {
        int num_neighs = vertex_web[current_vertex].num_neigh;
        int i = rand() % num_neighs;
        next_vertex = vertex_web[current_vertex].id_neigh[i];
        ROS_INFO("Choosing another random vertex %d\n", next_vertex);
    }
    
    // Look for any random vertex different from the previous one
    while (next_vertex == prev_vertex && next_vertex == current_vertex) {
        int i = rand() % dimension;
        next_vertex = i;
        ROS_INFO("Choosing another random vertex %d\n", next_vertex);
    }

    //Send the goal to the robot (Global Map)
    ROS_INFO("Re-Sending NEW goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    // calculate path to goal
    dijkstra(current_vertex, next_vertex, path_to_goal, to_goal_size, vertex_web, dimension);
    to_goal_position = 0;

    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(path_to_goal[to_goal_position+1]);
    
    goal_complete = false;    
}
int main(int argc, char** argv) {

    DTASSILearning_Agent agent;
    agent.init(argc,argv);
    agent.run();

    return 0;
}

