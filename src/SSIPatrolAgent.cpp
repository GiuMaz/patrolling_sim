#include "SSIPatrolAgent.h"

using namespace std;

bool* SSIPatrolAgent::create_selected_vertices(){
	bool* sv = new bool[dimension];
	for(size_t i=0; i<dimension; i++) {
	        selected_vertices[i] = false;
    	}	
	return sv;
}

void SSIPatrolAgent::reset_selected_vertices(bool* sv){
	for(size_t i=0; i<dimension; i++) {
	        sv[i] = false;
    	}
}

bool SSIPatrolAgent::all_selected(bool* sv){
	for(size_t i=0; i<dimension; i++) {
	        if (!sv[i]){ 
			return false;		
		}	
    	}
	return true;
}


void SSIPatrolAgent::init(int argc, char** argv) {
    
//   logfile = fopen("DTASSIOut.log","w");	
//    fprintf(logfile,"here0 \n");
//    fflush(logfile);
    
    PatrolAgent::init(argc,argv);
   
//    fprintf(logfile,"here1 \n");
//    fflush(logfile);

    //initialize structures
    taskRequests = new int[dimension];	
    tasks = new bool[dimension];	
    bids = new bid_tuple[dimension]; 
    global_instantaneous_idleness = new double[dimension];
    selected_vertices = new bool[dimension];
    bid_tuple noBid = {BIG_NUMBER,-1}; 
    for(size_t i=0; i<dimension; i++) {
	taskRequests[i] = 0;
        tasks[i] = false;
	selected_vertices[i] = false;
        bids[i] = noBid;
        global_instantaneous_idleness[i]=BIG_NUMBER;  // start with a high value    
    }
    last_update_idl = ros::Time::now().toSec();

//    fprintf(logfile,"here2 \n");
//    fflush(logfile);

    //initialize parameters
    timeout = cf.getDParam("timeout");
    theta_idl = cf.getDParam("theta_idleness");
    theta_cost = cf.getDParam("theta_navigation");
    threshold = cf.getDParam("threshold");			
    hist = cf.getDParam("hist");

//    printf("here3 \n");
//    fflush(stdout);

}

double SSIPatrolAgent::compute_cost(int vertex)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( current_vertex, vertex, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
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
        

//TODO this should give the geometric distance from robot position to next vertex, and not path cost from current_vertex to next_vertex
double SSIPatrolAgent::compute_distance(int vertex)
{

    //printf("TODO: implemente a function that gives geometric distance from current robot position to vertex"); 	

    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( current_vertex, vertex, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
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


double SSIPatrolAgent::compute_cost(int cv, int nv)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( cv, nv, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
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


double SSIPatrolAgent::utility(int cv,int nv) {
    double idl = global_instantaneous_idleness[nv];
    double cost = compute_cost(cv,nv);
    double U = theta_idl * idl + theta_cost * cost;
    //printf("   -- U[%d] ( %.1f, %.1f ) = %.1f\n",vertex,idl,cost,U);
    return U;
}

void SSIPatrolAgent::update_global_idleness() 
{   


    double now = ros::Time::now().toSec();
    
    for(size_t i=0; i<dimension; i++) {
        global_instantaneous_idleness[i] += (now-last_update_idl);  // update value    
    }
    
    last_update_idl = now;
}

int SSIPatrolAgent::select_next_vertex(int cv,bool* sv){
    
    double maxUtility = -1e9;
    int i_maxUtility = 0;


    //check if there are vertices not selected, if not reset all to false	
    if (all_selected(selected_vertices)) {
	reset_selected_vertices(sv);
    }
    for(size_t i=0; i<dimension; i++){
        
        double U = utility(cv,i);
        if (U > maxUtility && !sv[i]){
            maxUtility = U;
            i_maxUtility = i;
        }
    }
    
    int nv = i_maxUtility; // vertex_web[current_vertex].id_neigh[i_maxUtility];
    sv[nv] = true; //this vertex was considered as a next vertex
    printf("DTASSI: selected possible vertex = %d (U = %.2f)\n",nv,maxUtility);

    return nv;
}

double SSIPatrolAgent::compute_bid(int nv){

	printf("computing bid for vertex %d \n",nv);

	if (nv==next_vertex){
		printf("already going to %d sending 0 (current target: %d)",nv,next_vertex);
		return 0.;
	}

	//local copy of task list for computing the bid, put as selected the ones I do not want to consider 
	bool* my_tasks = new bool[dimension];
	//printf("current tasks [");
	for (size_t i = 0; i<dimension; i++){
	//	printf(" %d, ",tasks[i]);
		my_tasks[i] = !tasks[i];
	} 
	//printf(" ] \n");*/

	//add nv to my_tasks (regardless of whether this was my responsibility already)
	my_tasks[nv] = false;

/*	printf("my tasks [");
	for (size_t i = 0; i<dimension; i++){
		printf(" %d, ",my_tasks[i]);
	} 
	printf(" ] \n");
*/
	//accumulator for total path cost
	double path_cost = 0.;

	//put as first location the current target if any
	int ci = -1;
	if (next_vertex != -1){
		ci = next_vertex;
		path_cost = compute_distance(ci);//this should give the geometric distance from robot position to destination 
		my_tasks[ci] = true; //remove this task from the list
		//printf("[Target Set] Pathcost from %d to %d : %.2f \n",current_vertex,ci,path_cost);
	} else { 
		ci = select_next_vertex(current_vertex,my_tasks);
		path_cost = compute_cost(ci);
		//printf("[Target NOT Set] Pathcost from %d to %d : %.2f \n",current_vertex,ci,path_cost);
	}
	//handle remaining locations in reverse utility order
	while (!all_selected(my_tasks)){
		int ni = select_next_vertex(ci,my_tasks);
		path_cost += compute_cost(ci,ni);
		//printf("[while loop] pathcost from %d to %d : %.2f \n",ci,ni,path_cost);
		ci=ni;
	}
	printf("total cost = %.2f \n",path_cost);

	delete[] my_tasks;

	return path_cost;
}

void SSIPatrolAgent::force_bid(int nv,double bv,int rid){
	//printf("forcing bid for vertex %d with value %.2f from robot %d \n",nv,bv,rid);
	bids[nv].bidValue = bv;
	bids[nv].robotId = rid;
}

void SSIPatrolAgent::wait(){
	        ros::Duration delay = ros::Duration(timeout); //asynchronous version
	        delay.sleep();	
/*        double micro_timeout = timeout/ts; //synchronous version
	for (int i=0;i<ts;i++){
		// ros::spinOnce();
	        ros::Duration delay = ros::Duration(micro_timeout);
	        delay.sleep();	
	}
*/
}

// current_vertex (goal just reached)
int SSIPatrolAgent::compute_next_vertex() {

    update_global_idleness();
    global_instantaneous_idleness[current_vertex] = 0.0;

    reset_selected_vertices(selected_vertices);
    if (current_vertex >= 0 && current_vertex < dimension){
	selected_vertices[current_vertex] = true; //do not consider current vertex as possible goal 
    } 	
    int nv = select_next_vertex(current_vertex,selected_vertices);	
    double bidvalue = compute_bid(nv); 
    force_bid(nv,bidvalue,ID_ROBOT); 
    send_target(nv,bidvalue);
    printf("cnv: waiting for bids (%.2f seconds) \n",timeout);
    wait();	
    printf("current target %d current value for target %.2f \n tasks [",nv,bidvalue);
    for (size_t i = 0; i<dimension;i++){
	printf(" %d, ",tasks[i]);	
    }
    printf("] \n"); 
    while (true){
      if (best_bid(nv)){ //if I am in the best position to go to nv 
	update_tasks();
	//force_bid(nv,0,ID_ROBOT); TODO: check
	return nv;
      } else {
        nv = select_next_vertex(current_vertex,selected_vertices);	
	bidvalue = compute_bid(nv); 
	force_bid(nv,bidvalue,ID_ROBOT); 
	send_target(nv,bidvalue);
	printf("waiting for bids (%.2f seconds)",timeout);
	wait();
	printf("current target %d current value for target %.2f tasks [",nv,bidvalue);
        for (size_t i = 0; i<dimension;i++){
	    printf(" %d, ",tasks[i]);	
        }
        printf("] \n");
      } 			
    }     
}

void SSIPatrolAgent::update_tasks(){
    	/*printf("updating tasks: before: tasks [");
        for (size_t i = 0; i<dimension; i++){
	    printf(" %d, ",tasks[i]);	
        }
        printf("] \n"); 

    	printf("bids [");
        for (size_t i = 0; i<dimension; i++){
	    printf(" <%.2f,%d>, ",bids[i].bidValue,bids[i].robotId);	
        }
        printf("] \n"); 
*/
	for (size_t i = 0; i< dimension; i++){
		tasks[i] = (bids[i].robotId == ID_ROBOT);
	}
/*
    	printf("after [");
        for (size_t i = 0; i<dimension; i++){
	    printf(" %d, ",tasks[i]);	
        }
        printf("] \n"); 
*/
}

void SSIPatrolAgent::send_target(int nv,double bv) {
	//msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
    	int msg_type = DTASSI_TR;
    	std_msgs::Int16MultiArray msg;
    	msg.data.clear();

	msg.data.push_back(ID_ROBOT);
        msg.data.push_back(msg_type);
        msg.data.push_back(nv);
	msg.data.push_back(bv);
    	printf("  ** sending Task Request [%d, %d, %d, %.2f ] \n",ID_ROBOT,msg_type,nv,bv);
	results_pub.publish(msg);   
//    	ros::spinOnce();    
}


void SSIPatrolAgent::send_bid(int nv,double bv) {
	//msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
    	int msg_type = DTASSI_BID;
    	std_msgs::Int16MultiArray msg;
    	msg.data.clear();

	msg.data.push_back(ID_ROBOT);
        msg.data.push_back(msg_type);
        msg.data.push_back(nv);
	msg.data.push_back(bv);
    	printf("  ** sending Bid [%d, %d, %d, %.2f ] \n",ID_ROBOT,msg_type,nv,bv);
	results_pub.publish(msg);   
//    	ros::spinOnce();    
}


bool SSIPatrolAgent::best_bid(int nv){
	printf("computing whether I hold the best bid for %d, result: %d \n",nv,(bids[nv].robotId==ID_ROBOT));

/*    	printf("bids [");
        for (size_t i = 0; i<dimension; i++){
	    printf(" <%.2f,%d>, ",bids[i].bidValue,bids[i].robotId);	
        }
        printf("] \n"); 
*/

	return (bids[nv].robotId==ID_ROBOT);	
}

// current_vertex (goal just reached)
// next_vertex (next goal)
//make this blocking to wait for bids
//
void SSIPatrolAgent::send_results() {
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
    int msg_type = DTAGREEDY_MSG_TYPE;
    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT);
    msg.data.push_back(msg_type);
    printf("  ** sending [%d, %d, ",ID_ROBOT,msg_type);
    for(size_t i=0; i<dimension; i++) {
        // convert in 1/100 of secs (integer value)
        int ms = (int)(global_instantaneous_idleness[i]*100);
//        if ((int)i==next_vertex) ms=0; //sending 0 for next vertex to avoid conflicts (useless for DTASSI) TODO:CHECK 
        //printf("  ** sending GII[%lu] = %d\n",i,ms);
        printf("%d, ",ms);
        msg.data.push_back(ms);
    }
    msg.data.push_back(next_vertex);
    printf(",%d]\n",next_vertex);
    
    results_pub.publish(msg);   
    // ros::spinOnce();    
}

void SSIPatrolAgent::update_bids(int nv, double bv, int senderId){
	if (bids[nv].bidValue >= (1 + hist)*bv) { //using histeresis to avoid switching when there is no clear benefit
		bids[nv].bidValue = bv;
		bids[nv].robotId = senderId;
	}
	update_tasks();
}

void SSIPatrolAgent::idleness_msg_handler(std::vector<int>::const_iterator it){

    double now = ros::Time::now().toSec();

    for(size_t i=0; i<dimension; i++) {
	int ms = *it; it++; // received value
	// printf("  ** received from %d remote-GII[%lu] = %.1f\n",id_sender,i,ms);
	printf("%d, ",ms);
	double rgi = (double)ms/100.0; // convert back in seconds
	global_instantaneous_idleness[i] = std::min(
	    global_instantaneous_idleness[i]+(now-last_update_idl), rgi);
	// printf("   ++ GII[%lu] = %.1f (r=%.1f)\n",i,global_instantaneous_idleness[i],rgi);
    }
    last_update_idl = now;

/* TODO: CONSIDER INCLUDING THIS IN SSI AS A DIRECT DE-CONFLICTING PROCEDURE

		    int sender_next_vertex = *it; it++;
		    //printf("%d]\n",sender_next_vertex);
		    
		    // interrupt path if moving to the same target node
		    if (sender_next_vertex == next_vertex) { // two robots are going to the same node
			ROS_INFO("Robots %d and %d are both going to vertex %d",ID_ROBOT,id_sender,next_vertex);
			ROS_INFO("Robot %d: STOP and choose another target",ID_ROBOT);
			// change my destination
			cancelGoal(); // stop the current behavior
			current_vertex = next_vertex; // simulate that the goal vertex has been reached (not sent to the monitor)
			next_vertex = compute_next_vertex(); // compute next vertex (will be different from current vertex)
			sendGoal(next_vertex);
		    }
*/
}

void SSIPatrolAgent::task_request_msg_handler(std::vector<int>::const_iterator it, int senderId){
	int nv = *it; it++;
	double bv = *it; it++;
	printf("handling task request message: [ vertex: %d, bid value: %.2f]",nv,bv);
        double now = ros::Time::now().toSec();
	taskRequests[nv] = now;
	double my_bidValue = compute_bid(nv); 
	update_bids(nv,bv,senderId);
	if (my_bidValue<bv*(1+hist)){
		send_bid(nv,my_bidValue);
	}
}

void SSIPatrolAgent::bid_msg_handler(std::vector<int>::const_iterator it, int senderId){
	int nv = *it; it++;
	double bv = *it; it++;
	printf("handling bid message: [ vertex: %d, bid value: %.2f]",nv,bv);
	update_bids(nv,bv,senderId);
}


void SSIPatrolAgent::receive_results() {
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
        
    std::vector<int>::const_iterator it = vresults.begin();
    int id_sender = *it; it++;
    if (id_sender==ID_ROBOT) return;
    int msg_type = *it; it++;
    printf("  ** received [%d, %d, ... ] \n",id_sender,msg_type);
    switch (msg_type){
	    case (DTAGREEDY_MSG_TYPE):
	    {
	        idleness_msg_handler(it);
	    }
	    break;
	    case (DTASSI_TR):
	    {
		task_request_msg_handler(it,id_sender);
	    }	
	    break;			    
	    case (DTASSI_BID):
	    {
		bid_msg_handler(it,id_sender);
	    }	
	    break;
   }
}




