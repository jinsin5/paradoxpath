#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <list>
#include <cstdio>
#include <cstring>
#include <deque>

template<typename T, typename Number=int>
struct PriorityQueue {
    typedef std::pair<Number, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

    inline bool empty()
    {
        return elements.empty();
    }

    inline void put(
        T item,
        Number priority)
    {
        elements.emplace(priority, item);
    }

    inline T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

inline int heuristic(
    const int index0,
    const int index,
    const int nMapWidth)
{
    return std::abs((index0 % nMapWidth) - (index % nMapWidth)) +
           std::abs((index0 / nMapWidth) - (index / nMapWidth));
};

int
FindPath(
    const int nStartX,
    const int nStartY,
    const int nTargetX,
    const int nTargetY,
    const unsigned char* pMap,
    const int nMapWidth,
    const int nMapHeight,
    int* pOutBuffer,
    const int nOutBufferSize);

std::vector<int>
AdjacentNodes(
    const unsigned char* pMap,
    const int nMapWidth,
    const int nMapHeight,
    int cursor,
    int target);

int
FindPath(
    const int nStartX,
    const int nStartY,
    const int nTargetX,
    const int nTargetY,
    const unsigned char* pMap,
    const int nMapWidth,
    const int nMapHeight,
    int* pOutBuffer,
    const int nOutBufferSize)
{
    if (nMapWidth < 1 || nMapHeight < 1) {
        return -1;
    }

    if (nStartX < 0 || nTargetX < 0 || nMapWidth <= nTargetX || nMapWidth <= nStartX) {
        return -1;
    }

    if (nStartY < 0 || nTargetY < 0 || nMapHeight <= nTargetY || nMapHeight <= nStartY) {
        return -1;
    }

    if (nOutBufferSize < 0) {
        return -1;
    }

    int initial_start = nStartY * nMapWidth + nStartX;
    int initial_target = nTargetY * nMapWidth + nTargetX;

    PriorityQueue<int> pq_start;
    PriorityQueue<int> pq_end;

    std::unordered_map<int, int> starts_previous;
    std::unordered_map<int, int> ends_previous;

    std::unordered_map<int, int> starts_cost; // A
    std::unordered_map<int, int> ends_cost;  // D

    pq_start.put(initial_start, 0); // B
    pq_end.put(initial_target, 0);  // E

    starts_cost[initial_start] = 0;
    ends_cost[initial_target] = 0;

    int top_f; //length of the path from start to top element of forward heap.
    int top_r; //length of the path from target to top element of reverse heap.
    int mu = 100000; // Best path from start to end found. Initialize to arbitrary high number

    int current, target, best_current, best_target;

    std::vector<int> adj;

    /*Bidirectional A*
     *----------------
     */

    while (1) {
        current = pq_start.get();
        top_f = starts_cost[current];

        target = pq_end.get();
        top_r = ends_cost[target];
        /*
        if (top_f + top_r >= mu) {
            break;
        }*/

        if (top_f + heuristic(current, initial_target, nMapWidth) >= mu){
            break;
        }

        if (top_r + heuristic(target, initial_start, nMapWidth) >= mu){
            break;
        }

        //Forward Direction
        adj = AdjacentNodes(pMap, nMapWidth, nMapHeight, current, initial_target);

        for (std::vector<int>::size_type i = 0; i != adj.size() ; i++) {
            int new_cost = starts_cost[current] + 1; // weight is always 1
            if (!starts_cost.count(adj[i]) || new_cost < starts_cost[adj[i]]) {
                starts_cost[adj[i]] = new_cost;
                int priority = new_cost + heuristic(adj[i], initial_target, nMapWidth);
                pq_start.put(adj[i], priority);
                starts_previous[adj[i]] = current;

                // Update mu and current best source/target path vertices
                if (ends_cost.find(adj[i]) != ends_cost.end()) { // Vertice has been visited in reverse direction
                    if (starts_cost[current] + ends_cost[adj[i]] + 1 < mu) { // New best distance
                        mu = starts_cost[current] + ends_cost[adj[i]];
                        best_current = current;
                        best_target = adj[i];
                    }
                }
            }
        }

        if (pq_start.empty()){
            return -1;
        }

        // Reverse direction
        adj = AdjacentNodes(pMap, nMapWidth, nMapHeight, target, initial_target);

        for (std::vector<int>::size_type i = 0; i != adj.size() ; i++) {
            int new_cost = ends_cost[target] + 1; // weight is always 1
            if (!ends_cost.count(adj[i]) || new_cost < ends_cost[adj[i]]) {
                ends_cost[adj[i]] = new_cost;
                int priority = new_cost + heuristic(adj[i], initial_start, nMapWidth);
                pq_end.put(adj[i], priority);
                ends_previous[adj[i]] = target;
                // Update mu and current best source/target path vertices
                if (starts_cost.find(adj[i]) != starts_cost.end()){ // Vertice has been visited in forward direction
                    if (ends_cost[target] + starts_cost[adj[i]] + 1 < mu){ // New best distance
                        mu = ends_cost[target] + starts_cost[adj[i]];
                        best_current = adj[i];
                        best_target = target;
                    }
                }
            }
        }

        if (pq_end.empty()){
            return -1;
        }
    }

    if (best_current == initial_target){
        pOutBuffer[0] = best_target;
        return 1;
    } else if (best_target == initial_start) {
        pOutBuffer[0] = best_current;
        return 1;
    }

    int size = 2000000;
    int starts_path[size];
    int start_n = 0;

    while (best_current != initial_start){
        starts_path[start_n++] = best_current;
        best_current = starts_previous[best_current];
    }

    start_n--;
    int i = 0;
    while (1) {
        if (i+1 >= nOutBufferSize){
            return -1;
        }
        if (start_n >= 0){
            //printf("start adding: %d\n", starts_path[start_n]);
            pOutBuffer[i++] = starts_path[start_n--];
        } else {
            //printf("end adding: %d\n", best_target);
            pOutBuffer[i++] = best_target;
            best_target = ends_previous[best_target];
        }
        if (best_target == initial_target){
            break;
        }
    }
    pOutBuffer[i++] = best_target;

    return i;
}

std::vector<int>
AdjacentNodes(
    const unsigned char* pMap,
    const int nMapWidth,
    const int nMapHeight,
    int cursor,
    int target)
{
    int unfilteredAdjacent[] = {cursor + 1, cursor - 1,
                                cursor + nMapWidth, cursor - nMapWidth};

    // Prevent wrap arounds on the graph
    for (int i = 0; i < 2; i++){
        if ((unfilteredAdjacent[i]/nMapWidth) != (cursor/nMapWidth)){
            unfilteredAdjacent[i] = -1;
        }
    }

    std::vector<int> filteredAdjacent;

    for (int i = 0; i < 4; i++) {
        // In order: Check if we wrapped around, within the bounds of the array,
        // that the node adjacent is 1
        if (unfilteredAdjacent[i] != -1
                && unfilteredAdjacent[i] < (nMapWidth * nMapHeight)
                && ((int)pMap[unfilteredAdjacent[i]] == 1 || unfilteredAdjacent[i] == target)
                && unfilteredAdjacent[i] >= 0) {
            filteredAdjacent.push_back(unfilteredAdjacent[i]);

        }
    }

    return filteredAdjacent;
}
/*
int
main(
    int argc,
    char **argv)
{
    unsigned char pMap[] = {
        0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1,
        1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1,
        0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1,
        1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1,
        0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1,
        0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1,
        0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1,
        0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0
    };

    unsigned char pMap2[] = {1, 1, 1, 1,
                             0, 1, 0, 1,
                             0, 1, 1, 1};

    unsigned char pMap3[] = {0, 0, 1,
                             0, 1, 1,
                             1, 0, 1};

    int size = 50;
    int pOutBuffer[size];

    int result = FindPath(0, 0, 17, 13, pMap, 18, 14, pOutBuffer, size);
    //int result = FindPath(0, 0, 1, 2, pMap2, 4, 3, pOutBuffer, size);
    //int result = FindPath(2, 0, 0, 2, pMap3, 3, 3, pOutBuffer, size);

    std::cout << "Result: ";
    std::cout << result;

    if (result == -1){
        result = size;
    }
    std::cout << "\npOutBuffer: {";
    for (int i=0; i<result; i++) {
        std::cout << pOutBuffer[i];
        if (i != result-1) {
            std::cout << ", ";
        }
    }
    std::cout << "}\n";
}*/