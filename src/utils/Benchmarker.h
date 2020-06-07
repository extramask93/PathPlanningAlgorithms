//
// Created by damian on 04.05.2020.
//

#ifndef PLANNING_BENCHMARKER_H
#define PLANNING_BENCHMARKER_H

#include <vector>
#include <list>
#include <iomanip>
#include <numeric>
#include <fstream>
#include <IPlanner.h>
#include <Point.h>
#include <GridMap.h>
namespace util {
struct BenchmarkResult
{
    bool reachedGoal = 0;
    double pathLength = 0.0;
    double cputimems = 0.0;
    double cputimes = 0.0;
    int nrOfTurns = 0;
    friend std::ostream &operator<<(std::ostream &os, const BenchmarkResult &result)
    {
        os << std::setprecision(4) << result.reachedGoal << " " << result.pathLength << " " << result.cputimems << " " << result.cputimes;
        os << " " << result.nrOfTurns;
        return os;
    }
};
class Benchmarker
{
  public:
    Benchmarker() = default;
    void start();
    BenchmarkResult stop(const std::vector<util::Point> &path_);
    void evaluatePlanner(IPlanner &planner, std::shared_ptr<GridMap <unsigned char>> map, const Point &start, const Point &goal);
    double evaluatePlanners(IPlanner &planner, std::shared_ptr<GridMap <unsigned char>> map,
        const std::vector<std::pair<util::Point,util::Point>> &points, int nrOfRuns);
    double pathLength(util::GridMap<unsigned char> *map, const std::vector<util::Point> &path) const;
    void printAverages();
    void save(const std::string &fileLoc);
    void save(int nr, const std::string &file);
    [[deprecated]] void saveVar(const std::string &file, const char *fmt...);
    template<typename... T>
    void saveVars(const std::string &file, const T &... elems);
  private:
    template<typename T>
    void print(std::ostream &os, const T &elem);


  private:
    std::clock_t startClock_;
    std::clock_t stopClock_;
    std::list<util::BenchmarkResult> results_;
};
template<typename T>
void Benchmarker::print(std::ostream &os, const T &elem)
{
    os << elem << ' ';
}
template<typename... T>
void Benchmarker::saveVars(const std::string &file, const T &... elems)
{
    double sumOfPaths =
        std::accumulate(results_.begin(), results_.end(), 0.0, [](const auto &a, const auto &b) { return a + b.pathLength; });
    double sumOfTimes =
        std::accumulate(results_.begin(), results_.end(), 0.0, [](const auto &a, const auto &b) { return a + b.cputimems; });
    auto avergaePath = sumOfPaths / results_.size();
    auto averageTimes = sumOfTimes / results_.size();
    std::ofstream saveFile(file, std::ios::app);
    if (!saveFile.is_open()) {
        throw std::runtime_error("Cant open file: " + file);
    }
    (void)std::initializer_list<int>{ (print(saveFile, elems), 0)... };
    saveFile << avergaePath << " " << averageTimes << "\n";
}

}// namespace util


#endif//PLANNING_BENCHMARKER_H
