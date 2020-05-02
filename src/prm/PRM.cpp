//
// Created by damian on 26.04.2020.
//

#include <random>
#include <iostream>
#include "PRM.h"

prm::Prm::Prm(const util::GridMap<int> &map, unsigned nrOfSamples) : ogm_(map), nrOfSamples_(nrOfSamples){

}

std::vector<util::Point> prm::Prm::generateSamples(unsigned int nrOfSamples) {
    static std::mt19937 randomGenerator(std::random_device{}());
    std::uniform_real_distribution<> distributionX(0,ogm_.getWorldWidth());
    std::uniform_real_distribution<> distributionY(0, ogm_.getWorldHeight());
    std::vector<util::Point> samples;
    for(unsigned sampleNr = 0; sampleNr < nrOfSamples; ) {
        util::Point point = util::Point{distributionX(randomGenerator), distributionY(randomGenerator)};
        util::Location location = ogm_.worldToMap(point);
        if(ogm_.isFree(location)) {
            samples.push_back(point);
            sampleNr++;
        }
    }
    //return std::vector<util::Point>{{0,0},{0,2},{0,4}, {0,6}, {2,6}, {4,6},{6,6}};
    return samples;
}

std::vector<std::vector<int>> prm::Prm::generateRoadMap(unsigned nrOfSamples) {

    using namespace Nabo;
    using namespace Eigen;;
    auto samples = generateSamples(nrOfSamples);
    samples.push_back(start_);
    samples.push_back(goal_);
    // space for samples in 2D
    MatrixXf M = MatrixXf(2, samples.size());
    for(int i = 0; i< samples.size(); i++) {
        M(0,i) = samples[i].x;
        M(1,i) = samples[i].y;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    // look for the 5 nearest neighbour of a the single-point query
    VectorXi indices;
    VectorXf dists;
    indices.resize(samples.size()-1);
    dists.resize(samples.size()-1);
    std::vector<std::vector<int>> edges(samples.size());
    for(int sampleIndex =0; sampleIndex < samples.size(); sampleIndex++) {
        /*here should be connecting via edges to the closes neighbors*/
        // = M.row(sampleIndex);
        VectorXf q = VectorXf(2,1);
        q(0,0) = samples[sampleIndex].x;
        q(1,0) = samples[sampleIndex].y;
        nns->knn(q, indices, dists, samples.size()-1);
        for(int neighborIndex = 0; neighborIndex < samples.size()-1; neighborIndex++) {
            if(sampleIndex == indices[neighborIndex]) {
                continue;
            }
            if(isCollision(samples[sampleIndex],samples[indices[neighborIndex]])) {
                continue;
            }
            if(edges[sampleIndex].size() > MAX_NR_OF_EDGES_PER_POINT) {
                break;
            }
            edges[sampleIndex].push_back(indices[neighborIndex]);
        }
    }
    samples_ = samples;
    return edges;
}



std::vector<util::Point>
prm::Prm::nearestNeighbors(util::Point point, std::vector<util::Point> samples, int nNeighbors) {
    std::sort(samples.begin(), samples.end(),
            [&](const util::Point &a, const util::Point &b) {
            auto d1 = ogm_.worldDistanceEuclidean(a, point);
            auto d2 = ogm_.worldDistanceEuclidean(b,point);
            return d1<d2;
    });
    return std::vector<util::Point>{samples.begin(), samples.begin()+nNeighbors};
}

bool prm::Prm::isCollision(const util::Point &from, const util::Point &to) const {

    auto distanceBetweenNodes = ogm_.worldDistanceEuclidean(from, to);
    if(distanceBetweenNodes > MAX_EDGE_LENGTH_) {
        return true;
    }
    double angleBetweenNodes = std::atan2(to.y - from.y, to.x - from.x);
    int maxNumberOfExpandSteps = std::floor(distanceBetweenNodes / ogm_.getResolution());
    for (int i = 0; i < maxNumberOfExpandSteps; i++)
    {
        double xIncrement =  i * ogm_.getResolution() * cos(angleBetweenNodes);
        double yIncrement =  i * ogm_.getResolution() * sin(angleBetweenNodes);
        util::Point increment {xIncrement, yIncrement};
        auto cell = ogm_.worldToMap(from + increment);
        if(!ogm_.isFree(cell)) {
            return true;
        }
    }
    return false;
}

std::vector<util::Point> prm::Prm::makePlan(const util::Point &start, const util::Point &goal) {
    start_ = start;
    goal_ = goal;
    auto roadMap = generateRoadMap(nrOfSamples_);
    auto startNode = util::Node(roadMap.size()-2,start.x, start.y, 0.0, -1);
    auto goalNode = util::Node(roadMap.size()-1, goal.x, goal.y, 0.0, -1);
    openList_.clear();
    closedList_.clear();
    openList_.insert(std::make_pair(startNode.id, startNode));
    while(!openList_.empty()) {
        util::Node currentNode = std::min_element(openList_.begin(),openList_.end(),
                [](const auto &a, const auto &b) { return a.second.cost < b.second.cost;})->second;
        int currentID = currentNode.id;
        if(currentID == roadMap.size()-1) {
            /**goal**/
            goalNode = util::Node(currentID,goal.x, goal.y, currentNode.cost, currentNode.parent);
        }
        openList_.erase(currentID);
        closedList_.insert(std::make_pair(currentID, currentNode));

        for(int i = 0; i < roadMap[currentID].size(); i++) {
            int nextID = roadMap[currentID][i];
            double distance = ogm_.worldDistanceEuclidean(samples_[currentID], samples_[nextID]);
            util::Node node(nextID, samples_[nextID].x, samples_[nextID].y, distance, currentID);
            if(closedList_.count(nextID) > 0) {
                continue;
            }
            if(openList_.count(nextID) > 0 ) {
                auto it = openList_.find(nextID);
                if(it->second.cost > node.cost) {
                    it->second.cost = node.cost;
                    it->second.parent = currentID;
                }
            }
            else {
                openList_.insert(std::make_pair(nextID,node));
            }
        }
    }
    std::vector<util::Point> path;
    path.push_back({goalNode.x, goalNode.y});
    auto pind = goalNode.parent;
    while(pind != -1) {
        auto n = closedList_.find(pind);
        path.push_back({n->second.x,n->second.y});
        pind = n->second.parent;
    }
    return path;
}


