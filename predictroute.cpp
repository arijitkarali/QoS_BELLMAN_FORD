#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <list>

using namespace std;

#define nn 10      // number of nodes =10
#define simtime 20  // simulation time 10sec (1 unit time = 0.5 sec)
#define range 250  // Node wireless range = 250 co-ordinate units
#define INF 10000   // infinity = 10000
#define P_10_2 90   // 10c2 = 45

double initpos[10][2];
double initmove[10][3];
double velct_vector[10][2];   // 10 velocity vectors, each vector is of [x,y] form where v= xi+yj
int distMatrix[10][10];      // distance matrix
int routeupdates;
vector<int> route;
vector<int> visited(10,0);
int compsize,linkfails,routechngfreq,highchokefreq;
int chokecount[10];

typedef struct destination
{	
	vector<int> dist;
	vector<int> pred;
}destn;



void relax(int i, int j, int w, destn& v)
{

	if(v.dist[j] > v.dist[i] + w)
	{
		v.dist[j] = v.dist[i] + w;
		v.pred[j] = i;
	}
}


void buildroute(int i, int j, destn& v)
{

	if(v.pred[j]==-1)
	{
		string msg = "node number"+j;
		msg += "out of range";
		throw msg;
	}
	if(v.pred[j]==i)
	{
		route.push_back(j);
		return;
	}
	
	buildroute(i,v.pred[j],v);

	route.push_back(j);
	
	return;
}

vector<vector<int>> convert(int adj[][10])
{
    vector<vector<int>> adjList(nn*nn);
    for (int i = 0; i < nn; i++)
    {
          
        for (int j = 0; j < nn; j++)
        {
            if (adj[i][j] == 1)
            {
                adjList[i].push_back(j);
            }
        }
    }
    return adjList;
}

int measureComponent(int node,int v,vector<vector<int>>& adj)
{
	visited[v] = 1;
	compsize++;
    	vector<int>::iterator i;
    	if(v!=node){
    		for (i = adj[v].begin(); (i != adj[v].end()) && ((*i)!=node); ++i)
        		if (!visited[*i])
            			measureComponent(node,*i,adj);
            	}
       return compsize;
}



void QoS_BELLMAN_FORD(int source,int dest,int t){
	int i,j,adjacency[10][10];
	
	destn d; 
	d.dist = {INF,INF,INF,INF,INF,INF,INF,INF,INF,INF};
	d.pred = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
	d.dist[source]=0;
	vector<int> prevroute = route;
	
	for(i=0;i<nn;i++)
	{
		for(j=0;j<nn;j++)
			adjacency[i][j] = (distMatrix[i][j] == 0 || distMatrix[i][j] == INF)? 0 : 1;
	}
		
	
	list<int> queue;
	int s;
	queue.push_back(source);
	bool relaxed[10][10] = {false}; 
	
	//relaxing edges in bfs manner
	while(!queue.empty())
	{
		s = queue.front();
		queue.pop_front();
	 
		for (i = 0; i<nn ; ++i)
		{
			if (adjacency[s][i] && (!relaxed[s][i]))
			{
				relaxed[s][i] = true;
				queue.push_back(i);
				relax(s,i,distMatrix[s][i],d);
			}
		}
	 }
	
	try
	{
		route.clear();
		buildroute(source,dest,d);
		
		if(!routeupdates)
		{
			cout << "at t=" << t/2.0 <<" route established: "<<source;
			for (auto i : route)
				cout << "->"<< i;
			cout << endl;
			++routeupdates;	
		}
		else if ((prevroute != route) && routeupdates){
			++routechngfreq;
			cout << "at t=" << t/2.0 <<" predicted route update: "<<source;
			for (auto i : route)
				cout << "->"<< i;
			cout << endl;
			++routeupdates;
		}
		
		vector<vector<int>> adjlist = convert(adjacency);
		int v;
		float mcp;            //maxchokeprobability
		for(auto n : route)
		{
			compsize=0;
			fill(visited.begin(),visited.end(),0);
			while(n==(v=(rand()%10)));
			int m = measureComponent(n,v,adjlist);
			mcp= (float)((m+1)*(9-m)+(m)*(10-m))/(float)P_10_2;
			if(mcp>0.5)
			{
				++highchokefreq;
				chokecount[n]++;
				cout<<endl<<"at t="<<t/2.0 <<" at node = "<<n<<": high traffic choke is possible with prob. = "<<mcp<<endl; 
			}
			else if(mcp<0.5 && mcp>0.33)
			{
				cout<<endl<<"at t="<<t/2.0<<" at node = "<<n<<": moderate traffic choke predicted with prob. = "<<mcp<<endl;
			}
		}
	}
	
	catch(const string msg)
	{
		cout << msg << " at time =" << t*2 << "seconds" <<endl;
	}
}




int main()
{
	int i,source, dest;
	ifstream fin1,fin2;
	fin1.open("initialpos.txt");
	fin2.open("initialmovement.txt");
	
	while(fin1.get()!='#');   // ignoring comments and reaching input start character (#)
	for(i=0;i<nn;i++) fin1 >> initpos[i][0] >> initpos[i][1];
	
	while(fin2.get()!='#');
	for(i=0;i<nn;i++) fin2 >> initmove[i][0] >> initmove[i][1] >> initmove[i][2];
	
	fin1.close();
	fin2.close();
	
	double rel_x,rel_y,speed,abs_dist;
	for(i=0;i<nn;i++)               // movement information is processed here
	{
		rel_x = initmove[i][0]-initpos[i][0];
		rel_y = initmove[i][1]-initpos[i][1];
		abs_dist = sqrt(pow(rel_x,2)+pow(rel_y,2));
		speed = (initmove[i][2])/2;              // speed is taken per 0.5 sec
		velct_vector[i][0] = speed*rel_x/abs_dist;     // velocity vector along x
		velct_vector[i][1] = speed*rel_y/abs_dist;     // velocity vector along y
		
		//cout <<"node "<<i<<": "<<velct_vector[i][0]<<" "<<velct_vector[i][1]<< endl;
	}
	
	// initialize cost matrix based on global link states calculated from pos, vel informations
	double realtimepos[10][2];
	for(i=0;i<nn;i++) {
		realtimepos[i][0] = initpos[i][0];        //copying
		realtimepos[i][1] = initpos[i][1];
	}
	
	int j,k;
	cout<<"enter source node and destination node respectively"<<endl;
	cin >> source >> dest;
	
	for(i=0;i<simtime;i++)
	{
		// 20 times i.e. for 10 seconds , predict optimal route and predict failure
		for(j=0;j<nn;j++){
			realtimepos[j][0]+= velct_vector[j][0];  
			realtimepos[j][1]+= velct_vector[j][1];
		}
		
		float x_dist,y_dist;
		int dist;
		for(j=0;j<nn;j++)
		{
			for(k=j;k<nn;k++)
			{
				x_dist = realtimepos[j][0]-realtimepos[k][0];
				y_dist = realtimepos[j][1]-realtimepos[k][1];
				dist = (int)sqrt(pow(x_dist,2)+pow(y_dist,2));
				(dist<range)? distMatrix[j][k]=dist : distMatrix[j][k] = INF;
				distMatrix[k][j] = distMatrix[j][k];
			}
		}
		
		QoS_BELLMAN_FORD(source,dest,i);
		
		
			
	}
	int maxchokenode=-1,maxchokecount = 0;
	for(i=0;i<10;i++)
	{
		if(chokecount[i]>maxchokecount){
			maxchokecount = chokecount[i];
			maxchokenode=i;
		}
	}
	cout <<endl<<endl<<"\t-------ESTIMATE SUMMARY-------- " <<endl;
	cout << "\tQoS_BELLMAN_FORD estimates QoS affecting parameters for next 10 seconds as follows: " <<endl;
	cout << "\troutechange frequency:(higher routechanges leads to small packet drop): "<<((float)(routechngfreq-1))/10.0 <<" /sec"<<endl;
	cout << "\tpredicted end to end link failure instances: "<<linkfails <<endl;
	cout << "\thighchokefreq (number of times high traffic choking predicted): "<< highchokefreq <<" /10 sec" <<endl;
	if(maxchokenode>=0)cout << "\thighest choke, found at node: "<<maxchokenode<<endl;
	cout<< endl;
	return 0;
}
