//
// Created by damian on 02.05.2020.
//

#include <numeric>
#include <random>
#include <iostream>
#include "AntColony.h"
namespace ants {

AntColony::AntColony(const util::GridMap<unsigned char> &obstacleMap) : obstacleMap_(obstacleMap)
{
}
std::vector<util::Point> ants::AntColony::makePlan(const util::Point &start, const util::Point &goal)
{
    auto startLocation = obstacleMap_.worldToMap(start);
    auto goalLocation = obstacleMap_.worldToMap(goal);
    std::vector<util::Node<int>> lastBestPath{};
    initializePheromonesAtEdges();

    startNode_ = nodes_[obstacleMap_.mapToIndex(startLocation)];
    goalNode_ = nodes_[obstacleMap_.mapToIndex(goalLocation)];

    for (std::size_t iterNr = 0; iterNr < maxIterations_; iterNr++) {
        for (std::size_t antNrk = 0; antNrk < numOfAnts_; antNrk++) {
            /*all ants start in the anthill*/
            Ant ant{ startNode_, goalNode_ };
            while (!ant.foundGoal()) {
                auto newNode = selectNewNode(ant.getCurrentNode());
                ant.moveTo(newNode);
                ant.checkIfGoalReached();
            }
            ant.removeLoop();
            paths_.push_back(ant.getPath());
        }
        deterioratePheromones();
        lastBestPath = paths_[0];
        paths_.clear();
        std::cout<<"Iteration: "<<iterNr<<"length of the path: "<<lastBestPath.size()<<"\n";
        //break;
        }
        std::vector<util::Point> path(lastBestPath.size());
        std::transform(lastBestPath.begin(),lastBestPath.end(),path.begin(), [&](const auto &element){
            util::Location loc(element.x,element.y);
            return obstacleMap_.mapToWorld(loc);
        });
        return path;

    }



void AntColony::initializePheromonesAtEdges()
{
    int mapSize = obstacleMap_.getCellWidth()*obstacleMap_.getCellHeight();
    nodes_.clear();
    nodes_.reserve(mapSize);
    for (int i = 0 ;i < mapSize; i++) {
        auto location = obstacleMap_.indexToMap(i);
        auto node = util::Node(i, location.x, location.y);
        for (const auto &movement : util::Robot::getMotionModel()) {
            util::Location newLocation = location + movement;
            if (obstacleMap_.isWithinMapBounds(newLocation) && !isObstacle(newLocation) ) {
                util::Edge edge(newLocation,0,0);
                node.edges.emplace_back(edge);
            }
        }
        nodes_.emplace_back(node);
    }
}
util::Node<int>& AntColony::selectNewNode(util::Node<int> &node)
{
    double pheromoneSum =std::accumulate(node.edges.begin(),node.edges.end(), 0.0,
        [](double a, const util::Edge &b){return a+b.pheromone;});
    std::vector<util::Edge> edgesList;
    std::vector<double> probabilities;
    for(auto &edge: node.edges) {
        double probability = edge.pheromone / pheromoneSum;
        if(pheromoneSum ==0) {
            probability = 1.0 / node.edges.size();
        }
        edge.probability = probability;
        edgesList.push_back(edge);
        probabilities.push_back(probability);
    }
    std::transform(node.edges.begin(),node.edges.end(),node.edges.begin(),
        [](auto &a){a.probability = 0.0; return a;});

    static std::mt19937 randomGenerator(std::random_device{}());
    std::discrete_distribution<unsigned int> positionDistribution{ probabilities.begin(), probabilities.end() };
    auto finalNodeLocation = edgesList.at(positionDistribution(randomGenerator)).finalNode;
    return nodes_[obstacleMap_.mapToIndex(finalNodeLocation)];

}


void AntColony::deterioratePheromones()
{
    std::sort(paths_.begin(), paths_.end(),[](const auto &vec1, const auto &vec2) {return vec1.size()< vec2.size();});
    for(int pathIndex =0; pathIndex < paths_.size(); pathIndex++) {
        for(int elementIndex =0; elementIndex < paths_.at(pathIndex).size()-1; elementIndex++) {
            auto currentNode = paths_[pathIndex][elementIndex];
            auto iter = std::find_if(nodes_.begin(),nodes_.end(), [&](const auto &a){return a.id == currentNode.id;});
            for(auto &edge: iter->edges) {
                if(obstacleMap_.mapToIndex(edge.finalNode) == paths_.at(pathIndex).at(elementIndex+1).id) {
                    edge.pheromone = (1.0 - evaporationRate_) * edge.pheromone + (rewardScalingFactor_ / paths_.at(pathIndex).size());
                }
                else {
                    edge.pheromone = (1.0 - evaporationRate_) * edge.pheromone ;
                }
            }
        }
    }
}


bool AntColony::isObstacle(const util::Location &location) const
{
    return obstacleMap_[location] == 0;
}

}// namespace ants
