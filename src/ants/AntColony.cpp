

#include <random>
#include <Options.h>
#include <climits>
#include "AntColony.h"

Ant::Ant(Node start, int id)
{
    this->id_ = id;
    this->current_node_ = start;
    this->previous_node_ = Node(-1, -1);
}

AntColony::AntColony(std::shared_ptr<util::GridMap<unsigned char>> grid,
    int n_ants, double alpha, double beta, double evap_rate, int iterations, double Q) : IPlanner(grid)

{
    this->n_ants_ = n_ants;
    this->alpha_ = alpha;
    this->beta_ = beta;
    this->evap_rate_ = evap_rate;
    this->iterations_ = iterations;
    this->ants_ = std::vector<Ant>(n_ants_);
    this->Q_ = Q;
}

void AntColony::RemoveLoop(Ant &ant)
{
    for (auto it = ant.path_.begin(); it != ant.path_.end(); ++it) {
        if (*it == ant.current_node_) {
            ant.steps_ = ant.path_.end() - it;
            ant.path_.erase(it, ant.path_.end());
            break;
        }
    }
}

std::vector<Node> AntColony::ant_colony(util::GridMap<unsigned char> &grid, Node start, Node goal)
{
    this->start_ = start;// Make sure start has id
    this->goal_ = goal;
    grid_size_ = map_->getCellWidth();
    Node c;
    motions_ = GetMotion();
    for (int i = 0; i < grid_size_; i++) {
        for (int j = 0; j < grid_size_; j++) {
            for (auto &motion : motions_) {
                c = Node(i, j) + motion;
                if (c.x_ >= 0 && c.x_ < grid_size_ && c.y_ >= 0 && c.y_ < grid_size_) pheromone_edges_.insert({ std::make_pair(i * grid_size_ + j, c.x_ * grid_size_ + c.y_), 1.0 });
            }
        }
    }
    this->max_steps_ = pow(grid_size_, 2) / 2 + grid_size_;

    std::random_device device;
    std::mt19937 engine(device());
    std::vector<Node> last_best_path;// saves best path of last iteration. Not over all.
    Node possible_position;

    for (int i = 0; i < iterations_; i++) {
        for (int j = 0; j < n_ants_; j++) {
            // Can assign a thread to each ant if parallel required
            Ant ant(start_, j);
            while (ant.current_node_ != goal_ && ant.steps_ < max_steps_) {
                ant.path_.push_back(ant.current_node_);

                // Get next position
                std::vector<Node> possible_positions;
                std::vector<double> possible_probabilities;
                float prob_sum = 0;
                int n_obs = 0;
                for (auto &m : motions_) {
                    possible_position = ant.current_node_ + m;
                    possible_position.id_ = possible_position.x_ * grid_size_ + possible_position.y_;
                    if (possible_position.x_ >= 0 && possible_position.x_ < grid_size_ && possible_position.y_ >= 0 && possible_position.y_ < grid_size_
                        && possible_position != ant.previous_node_) {
                        possible_positions.push_back(possible_position);
                        double new_prob = pow(pheromone_edges_[std::make_pair(possible_position.id_, ant.current_node_.id_)], alpha_) * pow(1.0 / pow(pow((possible_position.x_ - goal_.x_), 2) + pow((possible_position.y_ - goal_.y_), 2), 0.5), beta_);
                        if ((*map_)[util::Location{ possible_position.x_, possible_position.y_ }] == 0) {
                            n_obs += 1;
                            new_prob = 0;
                        }
                        possible_probabilities.push_back(new_prob);
                        prob_sum += new_prob;
                    }
                }
                if (n_obs == possible_positions.size())
                    break;// Ant in a cul-de-sac
                else if (prob_sum == 0) {
                    double new_prob = 1.0 / (possible_positions.size() - n_obs);
                    for (int i = 0; i < possible_positions.size(); i++) {
                        if ((*map_)[util::Location{ possible_positions[i].x_, possible_positions[i].y_ }] == 1)
                            possible_probabilities[i] = new_prob;
                        else
                            possible_probabilities[i] = 0;
                    }
                } else
                    for (auto &p : possible_probabilities) p /= prob_sum;
                std::discrete_distribution<> dist(possible_probabilities.begin(), possible_probabilities.end());
                ant.previous_node_ = ant.current_node_;
                ant.current_node_ = possible_positions[dist(engine)];
                RemoveLoop(ant);

                ant.steps_++;
            }
            // If goal found, add to path
            if (ant.current_node_ == goal_) {
                ant.current_node_.id_ = ant.current_node_.x_ * grid_size_ + ant.current_node_.y_;
                ant.path_.push_back(ant.current_node_);
                ant.found_goal_ = true;
            }
            ants_[j] = ant;
        }

        // Pheromone deterioration
        for (auto it = pheromone_edges_.begin(); it != pheromone_edges_.end(); it++) it->second = it->second * (1 - evap_rate_);

        int bpl = INT_MAX;
        std::vector<Node> bp;

        // Pheromone update based on successful ants
        for (Ant &ant : ants_) {
            // PrintAntPath(ant);
            if (ant.found_goal_) {// Use iff goal reached
                if (ant.path_.size() < bpl) {// Save best path yet in this iteration
                    bpl = ant.path_.size();
                    bp = ant.path_;
                }
                double c = Q_ / (ant.path_.size() - 1);//c = cost / reward. Reward here, increased pheromone
                for (int i = 1; i < ant.path_.size(); i++) {// Assuming ant can tell which way the food was based on how the phermones detereorate. Good for path planning as prevents moving in the opposite direction to path and improves convergence
                    auto it = pheromone_edges_.find(std::make_pair(ant.path_[i].id_, ant.path_[i - 1].id_));
                    it->second += c;
                }
            }
        }
        if (i + 1 == iterations_) last_best_path = bp;
    }//for every iteration loop ends here
    if (last_best_path.empty()) {
        last_best_path.clear();
        Node no_path_node(-1, -1, -1, -1, -1, -1);
        last_best_path.push_back(no_path_node);
        return last_best_path;
    }
    for (int i = 1; i < last_best_path.size(); i++) last_best_path[i].pid_ = last_best_path[i - 1].id_;
    last_best_path.back().id_ = last_best_path.back().x_ * grid_size_ + last_best_path.back().y_;
    return last_best_path;
}
std::vector<util::Point> AntColony::makePlan(const util::Point &start_, const util::Point &goal_)
{
    Node start(map_->worldToMap(start_).x, map_->worldToMap(start_).y, 0, 0, 0, 0);
    Node goal(map_->worldToMap(goal_).x, map_->worldToMap(goal_).y, 0, 0, 0, 0);
    this->ants_ = std::vector<Ant>(n_ants_);

    start.id_ = start.x_ * map_->getCellWidth() + start.y_;
    start.pid_ = start.x_ * map_->getCellWidth() + start.y_;
    goal.id_ = goal.x_ * map_->getCellWidth() + goal.y_;
    start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
    std::vector<Node> path = ant_colony(*map_, start, goal);
    if (path[0].id_ == -1) {
        return std::vector<util::Point>();
    }
    std::vector<util::Point> pointPath;
    for (auto node : path) {
        auto point = util::Point(map_->mapToWorld(util::Location(node.x_, node.y_)));
        pointPath.push_back(point);
    }
    return pointPath;
}

