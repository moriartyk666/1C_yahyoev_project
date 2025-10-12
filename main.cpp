#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Структура для позиции с оператором сравнения.
struct Position {
  int64_t x;
  int64_t y;

  // Оператор сравнения для unordered_set и GetPath.
  bool operator==(const Position& other) const {
    return x == other.x && y == other.y;
  }
};

// Хэш для структуры Position.
struct HashPosition {
  std::size_t operator()(const Position& p) const {
    auto hash_x = std::hash<int64_t>{}(p.x);
    auto hash_y = std::hash<int64_t>{}(p.y);
    return hash_x ^ (hash_y << 1);  // Простое комбинирование хэшей.
  }
};

// Хэш для State
struct HashState {
  std::size_t operator()(const std::pair<Position, int64_t>& p) const {
    auto hash_pos = HashPosition{}(p.first);
    auto hash_dir = std::hash<int64_t>{}(p.second);
    return hash_pos ^ (hash_dir << 1);
  }
};

using State = std::pair<Position, int64_t>;  // {позиция, направление}.

// Класс для решения задачи обхода лабиринта.
class MazeSolver {
 public:
  MazeSolver(const Position& start_position, int64_t start_direction,
             int64_t step_cost, int64_t turn_cost, int64_t scout_cost,
             int64_t scout_range)
      : current_position_(start_position),
        current_direction_(start_direction),
        step_cost_(step_cost),
        turn_cost_(turn_cost),
        scout_cost_(scout_cost),
        scout_range_(scout_range),
        total_time_(0) {
    known_floors_.insert(start_position);
    visited_positions_.insert(start_position);
  }

  // Основной метод решения.
  void Solve() {
    if (scout_cost_ < 4 * step_cost_) {
      DoScout();
    }

    while (!IsComplete()) {
      std::unordered_map<State, State, HashState> parent;
      State start_state = {current_position_, current_direction_};
      auto distances = ComputeDists(start_state, known_floors_, step_cost_,
                                    turn_cost_, parent);

      auto visit_targets = FindVisitTargets(distances);
      auto probe_targets = FindProbeTargets(distances);

      auto [best_cost, is_visit, target_position, target_probe] =
          SelectBestTarget(visit_targets, probe_targets);
      if (best_cost == std::numeric_limits<int64_t>::max()) {
        DoScout();
        continue;
      }
      if (best_cost == 0 && is_visit) {
        continue;  // Уже на месте.
      }

      State target_state = is_visit
                               ? GetBestVisitState(target_position, distances)
                               : target_probe;

      auto actions = GetPath(target_state, parent);
      ExecutePath(actions);
      ExecuteTarget(is_visit, target_position, target_probe);
    }
  }

  // Получение итогового времени.
  int64_t GetTotalTime() const { return total_time_; }

 private:
  // Константы направлений: 0:вверх, 1:направо, 2:вниз, 3:влево.
  static const std::vector<std::pair<int64_t, int64_t>> kDirections;

  // Проверка завершения обхода.
  bool IsComplete() const {
    if (visited_positions_.size() != known_floors_.size()) {
      return false;
    }
    for (const auto& pos : known_floors_) {
      for (int64_t dir = 0; dir < 4; ++dir) {
        Position next = {pos.x + kDirections[dir].first,
                         pos.y + kDirections[dir].second};
        State probe = {pos, dir};
        if (known_floors_.find(next) == known_floors_.end() &&
            bumped_positions_.find(probe) == bumped_positions_.end()) {
          return false;
        }
      }
    }
    return true;
  }

  // Выполнение разведки (костёр).
  void DoScout() {
    std::cout << 3 << std::endl;
    total_time_ += scout_cost_;
    int64_t size = 2 * scout_range_ + 1;
    for (int64_t i = 0; i < size; ++i) {
      std::string row;
      std::cin >> row;
      int64_t dy = scout_range_ - i;
      for (int64_t j = 0; j < size; ++j) {
        int64_t dx = j - scout_range_;
        if (row[j] == '_') {
          Position new_pos = {current_position_.x + dx,
                              current_position_.y + dy};
          known_floors_.insert(new_pos);
        }
      }
    }
    for (int64_t dir = 0; dir < 4; ++dir) {
      Position adj = {current_position_.x + kDirections[dir].first,
                      current_position_.y + kDirections[dir].second};
      if (known_floors_.find(adj) == known_floors_.end()) {
        bumped_positions_.insert({current_position_, dir});
      }
    }
  }

  // Алгоритм Дейкстры для вычисления минимальных стоимостей до всех состояний.
  static std::unordered_map<State, int64_t, HashState> ComputeDists(
      const State& start_state,
      const std::unordered_set<Position, HashPosition>& known_floors,
      int64_t step_cost, int64_t turn_cost,
      std::unordered_map<State, State, HashState>& parent) {
    std::unordered_map<State, int64_t, HashState> dist;
    auto cmp = [](const std::pair<int64_t, State>& lhs,
                  const std::pair<int64_t, State>& rhs) {
      return lhs.first > rhs.first;
    };
    std::priority_queue<std::pair<int64_t, State>,
                        std::vector<std::pair<int64_t, State>>, decltype(cmp)>
        pq(cmp);

    dist[start_state] = 0;
    pq.push({0, start_state});

    while (!pq.empty()) {
      auto [cost, state] = pq.top();
      pq.pop();
      if (dist.find(state) != dist.end() && cost > dist[state]) {
        continue;
      }

      Position pos = state.first;
      int64_t dir = state.second;

      int64_t left_dir = (dir - 1 + 4) % 4;
      State left_state = {pos, left_dir};
      int64_t new_cost = cost + turn_cost;
      if (dist.find(left_state) == dist.end() || new_cost < dist[left_state]) {
        dist[left_state] = new_cost;
        parent[left_state] = state;
        pq.push({new_cost, left_state});
      }

      int64_t right_dir = (dir + 1) % 4;
      State right_state = {pos, right_dir};
      new_cost = cost + turn_cost;
      if (dist.find(right_state) == dist.end() ||
          new_cost < dist[right_state]) {
        dist[right_state] = new_cost;
        parent[right_state] = state;
        pq.push({new_cost, right_state});
      }

      Position next_pos = {pos.x + kDirections[dir].first,
                           pos.y + kDirections[dir].second};
      if (known_floors.find(next_pos) != known_floors.end()) {
        State next_state = {next_pos, dir};
        new_cost = cost + step_cost;
        if (dist.find(next_state) == dist.end() ||
            new_cost < dist[next_state]) {
          dist[next_state] = new_cost;
          parent[next_state] = state;
          pq.push({new_cost, next_state});
        }
      }
    }
    return dist;
  }

  // Восстановление пути от старта до цели.
  static std::vector<std::pair<int64_t, int64_t>> GetPath(
      const State& target,
      const std::unordered_map<State, State, HashState>& parent) {
    std::vector<std::pair<int64_t, int64_t>> actions;
    State current = target;
    while (true) {
      auto it = parent.find(current);
      if (it == parent.end()) {
        break;
      }
      State prev = it->second;
      if (prev.first == current.first) {
        int64_t turn_dir =
            (current.second == (prev.second - 1 + 4) % 4) ? 0 : 1;
        actions.push_back({2, turn_dir});
      } else {
        actions.push_back({1, -1});
      }
      current = prev;
    }
    std::reverse(actions.begin(), actions.end());
    return actions;
  }

  // Поиск целей для посещения.
  std::map<int64_t, std::vector<Position>> FindVisitTargets(
      const std::unordered_map<State, int64_t, HashState>& distances) const {
    std::map<int64_t, std::vector<Position>> visit_targets;
    for (const auto& pos : known_floors_) {
      if (visited_positions_.find(pos) != visited_positions_.end()) {
        continue;
      }
      int64_t min_distance = std::numeric_limits<int64_t>::max();
      for (int64_t dir = 0; dir < 4; ++dir) {
        State state = {pos, dir};
        auto it = distances.find(state);
        if (it != distances.end()) {
          min_distance = std::min(min_distance, it->second);
        }
      }
      if (min_distance != std::numeric_limits<int64_t>::max()) {
        visit_targets[min_distance].push_back(pos);
      }
    }
    return visit_targets;
  }

  // Поиск целей для проверки границ.
  std::map<int64_t, std::vector<State>> FindProbeTargets(
      const std::unordered_map<State, int64_t, HashState>& distances) const {
    std::map<int64_t, std::vector<State>> probe_targets;
    for (const auto& pos : known_floors_) {
      for (int64_t dir = 0; dir < 4; ++dir) {
        Position next = {pos.x + kDirections[dir].first,
                         pos.y + kDirections[dir].second};
        State probe = {pos, dir};
        if (known_floors_.find(next) == known_floors_.end() &&
            bumped_positions_.find(probe) == bumped_positions_.end()) {
          auto it = distances.find(probe);
          if (it != distances.end()) {
            int64_t probe_cost = it->second + step_cost_;
            probe_targets[probe_cost].push_back(probe);
          }
        }
      }
    }
    return probe_targets;
  }

  // Выбор лучшей цели.
  std::tuple<int64_t, bool, Position, State> SelectBestTarget(
      const std::map<int64_t, std::vector<Position>>& visit_targets,
      const std::map<int64_t, std::vector<State>>& probe_targets) const {
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    bool is_visit = false;
    Position target_position = {-1, -1};
    State target_probe = {{-1, -1}, -1};

    if (!visit_targets.empty()) {
      best_cost = visit_targets.begin()->first;
      target_position = visit_targets.begin()->second[0];
      is_visit = true;
    }
    if (!probe_targets.empty()) {
      int64_t probe_min = probe_targets.begin()->first;
      if (probe_min < best_cost || (probe_min == best_cost && !is_visit)) {
        best_cost = probe_min;
        target_probe = probe_targets.begin()->second[0];
        is_visit = false;
      }
    }
    return {best_cost, is_visit, target_position, target_probe};
  }

  // Выбор лучшего состояния для посещения.
  State GetBestVisitState(
      const Position& target_position,
      const std::unordered_map<State, int64_t, HashState>& distances) const {
    int64_t min_distance = std::numeric_limits<int64_t>::max();
    int64_t best_dir = 0;
    for (int64_t dir = 0; dir < 4; ++dir) {
      State state = {target_position, dir};
      auto it = distances.find(state);
      if (it != distances.end() && it->second < min_distance) {
        min_distance = it->second;
        best_dir = dir;
      }
    }
    return {target_position, best_dir};
  }

  // Выполнение пути к цели.
  void ExecutePath(const std::vector<std::pair<int64_t, int64_t>>& actions) {
    for (const auto& [action, param] : actions) {
      if (action == 1) {
        std::cout << 1 << std::endl;
        int64_t result;
        std::cin >> result;
        total_time_ += step_cost_;
        if (result == 1) {
          current_position_.x += kDirections[current_direction_].first;
          current_position_.y += kDirections[current_direction_].second;
          visited_positions_.insert(current_position_);
        }
      } else {
        std::cout << 2 << " " << param << std::endl;
        int64_t result;
        std::cin >> result;
        total_time_ += turn_cost_;
        current_direction_ = (param == 0) ? (current_direction_ - 1 + 4) % 4
                                          : (current_direction_ + 1) % 4;
      }
    }
  }

  // Выполнение действия на цели.
  void ExecuteTarget(bool is_visit, const Position& target_position,
                     const State& target_probe) {
    if (is_visit) {
      visited_positions_.insert(target_position);
      if (scout_cost_ < step_cost_ * scout_range_ / 2) {
        DoScout();
      }
    } else {
      std::cout << 1 << std::endl;
      int64_t result;
      std::cin >> result;
      total_time_ += step_cost_;
      if (result == 1) {
        current_position_.x += kDirections[current_direction_].first;
        current_position_.y += kDirections[current_direction_].second;
        known_floors_.insert(current_position_);
        visited_positions_.insert(current_position_);
        if (scout_cost_ < step_cost_ * scout_range_ / 2) {
          DoScout();
        }
      } else {
        bumped_positions_.insert(target_probe);
      }
    }
  }

  Position current_position_;
  int64_t current_direction_;
  const int64_t step_cost_;
  const int64_t turn_cost_;
  const int64_t scout_cost_;
  const int64_t scout_range_;
  int64_t total_time_;
  std::unordered_set<Position, HashPosition> known_floors_;
  std::unordered_set<Position, HashPosition> visited_positions_;
  std::unordered_set<State, HashState> bumped_positions_;
};

// Определение константы направлений.
const std::vector<std::pair<int64_t, int64_t>> MazeSolver::kDirections = {
    {0, 1}, {1, 0}, {0, -1}, {-1, 0}};

// Функция для получения направления.
int64_t GetDir(int64_t dx, int64_t dy) {
  if (dx == 0 && dy == 1) return 0;
  if (dx == 1 && dy == 0) return 1;
  if (dx == 0 && dy == -1) return 2;
  if (dx == -1 && dy == 0) return 3;
  return -1;
}

int main() {
  int64_t start_x;
  int64_t start_y;
  int64_t look_x;
  int64_t look_y;
  int64_t step_cost;
  int64_t turn_cost;
  int64_t scout_cost;
  int64_t scout_range;
  std::cin >> start_x >> start_y >> look_x >> look_y >> step_cost >>
      turn_cost >> scout_cost >> scout_range;

  Position start_position = {start_x, start_y};
  int64_t start_direction = GetDir(look_x - start_x, look_y - start_y);

  MazeSolver solver(start_position, start_direction, step_cost, turn_cost,
                    scout_cost, scout_range);
  solver.Solve();

  std::cout << 4 << " " << solver.GetTotalTime() << std::endl;
  return 0;
}