#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;

// ���尲ȫ����ṹ
struct Interval {
    int start;
    int end;

    Interval(int s, int e) : start(s), end(e) {}

    bool operator==(const Interval& other) const {
        return start == other.start && end == other.end;
    }
};

// ����ڵ�ṹ
struct Node {
    int x, y;           // �ڵ�����
    Interval interval;  // ��ȫ����
    int g;              // ����㵽��ǰ�ڵ��ʵ�ʴ���
    int h;              // ����ʽ���ƴ���
    int f;              // f = g + h
    Node* parent;       // ���ڵ�ָ��
    int arrival_time;   // ����ʱ��

    Node(int x, int y, Interval interval, int g, int h, Node* parent, int arrival)
        : x(x), y(y), interval(interval), g(g), h(h), f(g + h), parent(parent), arrival_time(arrival) {
    }

    // ����<������������ȶ���
    bool operator<(const Node& other) const {
        return f > other.f; // ע�⣺����Ϊ�˹�����С��
    }
};

// �������񻷾�
struct Grid {
    int width;
    int height;
    vector<vector<bool>> static_obstacles; // ��̬�ϰ����ͼ

    Grid(int w, int h) : width(w), height(h) {
        static_obstacles.resize(w, vector<bool>(h, false));
    }

    bool is_valid(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height && !static_obstacles[x][y];
    }
};

// ���嶯̬�ϰ���
struct DynamicObstacle {
    int x, y;           // �ϰ���λ��
    int start_time;     // ��ʼʱ��
    int end_time;       // ����ʱ��

    DynamicObstacle(int x, int y, int start, int end)
        : x(x), y(y), start_time(start), end_time(end) {
    }
};

// ���������پ�����Ϊ����ʽ����
int manhattan_distance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// ������������Ԫ�İ�ȫ����
unordered_map<int, unordered_map<int, vector<Interval>>> compute_safe_intervals(
    const Grid& grid,
    const vector<DynamicObstacle>& obstacles,
    int max_time) {

    unordered_map<int, unordered_map<int, vector<Interval>>> safe_intervals;

    // ��ʼ����������Ԫ�İ�ȫ����Ϊ[0, max_time]
    for (int x = 0; x < grid.width; ++x) {
        for (int y = 0; y < grid.height; ++y) {
            if (grid.is_valid(x, y)) {
                safe_intervals[x][y].emplace_back(0, max_time);
            }
        }
    }

    // ���ݶ�̬�ϰ�����°�ȫ����
    for (const auto& obs : obstacles) {
        int x = obs.x;
        int y = obs.y;

        if (!grid.is_valid(x, y)) continue;

        vector<Interval> new_intervals;
        for (const auto& interval : safe_intervals[x][y]) {
            // ����ϰ���ʱ���뵱ǰ�������ص�
            if (obs.end_time < interval.start || obs.start_time > interval.end) {
                new_intervals.push_back(interval);
            }
            // ����ϰ�����ȫ���ǵ�ǰ����
            else if (obs.start_time <= interval.start && obs.end_time >= interval.end) {
                continue; // �����������
            }
            // ����ϰ��ﲿ�ָ��ǵ�ǰ����
            else {
                // �������ϰ���֮ǰ�Ĳ���
                if (obs.start_time > interval.start) {
                    new_intervals.emplace_back(interval.start, obs.start_time - 1);
                }
                // �������ϰ���֮��Ĳ���
                if (obs.end_time < interval.end) {
                    new_intervals.emplace_back(obs.end_time + 1, interval.end);
                }
            }
        }
        safe_intervals[x][y] = new_intervals;
    }

    return safe_intervals;
}

// ���ʱ����Ƿ��ڰ�ȫ������
bool is_in_safe_interval(int time, const vector<Interval>& intervals) {
    for (const auto& interval : intervals) {
        if (time >= interval.start && time <= interval.end) {
            return true;
        }
    }
    return false;
}

// ���ҿ��Ե���Ŀ������ĵ���ʱ��
pair<bool, int> find_arrival_time(int start_time, int move_duration, const vector<Interval>& target_intervals) {
    for (const auto& interval : target_intervals) {
        int earliest_arrival = max(start_time + move_duration, interval.start);
        if (earliest_arrival <= interval.end) {
            return { true, earliest_arrival };
        }
    }
    return { false, -1 };
}

// SIPP�㷨������
vector<pair<int, int>> sipp(
    const Grid& grid,
    pair<int, int> start,
    pair<int, int> goal,
    const vector<DynamicObstacle>& obstacles,
    int max_time) {

    // ���㰲ȫ����
    auto safe_intervals = compute_safe_intervals(grid, obstacles, max_time);

    // ���ȶ��У������б�
    priority_queue<Node> open_list;

    // �ر��б���¼����չ�Ľڵ㣩
    unordered_map<int, unordered_map<int, unordered_map<Interval, bool,
        function<size_t(const Interval&)>>> closed_list;

    // ��ʼ�����
    if (safe_intervals[start.first][start.second].empty()) {
        return {}; // ���û�а�ȫ���䣬�޽�
    }

    // ʹ�õ�һ����ȫ������Ϊ���İ�ȫ����
    Interval start_interval = safe_intervals[start.first][start.second][0];
    int h = manhattan_distance(start.first, start.second, goal.first, goal.second);
    open_list.emplace(start.first, start.second, start_interval, 0, h, nullptr, 0);

    // �����ƶ������ϡ��¡����ҡ��ȴ���
    vector<pair<int, int>> directions = { {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {0, 0} };

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        // ����Ƿ񵽴�Ŀ��
        if (current.x == goal.first && current.y == goal.second) {
            // ����·��
            vector<pair<int, int>> path;
            Node* node = &current;
            while (node != nullptr) {
                path.emplace_back(node->x, node->y);
                node = node->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        // ����Ƿ��Ѿ��ڹر��б���
        if (closed_list[current.x][current.y][current.interval]) {
            continue;
        }
        closed_list[current.x][current.y][current.interval] = true;

        // ���ɺ�̽ڵ�
        for (const auto& dir : directions) {
            int new_x = current.x + dir.first;
            int new_y = current.y + dir.second;

            // �����λ���Ƿ���Ч
            if (!grid.is_valid(new_x, new_y)) continue;

            // �����ƶ�ʱ�䣨�ȴ�Ϊ1���ƶ�Ϊ1��
            int move_duration = (dir.first == 0 && dir.second == 0) ? 1 : 1;

            // ���Ŀ��λ�õİ�ȫ����
            for (const auto& target_interval : safe_intervals[new_x][new_y]) {
                // ������Ե���Ŀ����������絽��ʱ��
                auto [found, arrival_time] = find_arrival_time(
                    current.arrival_time, move_duration, { target_interval });

                if (found) {
                    // ����ƶ��������Ƿ���붯̬�ϰ�����ײ
                    bool collision = false;
                    for (const auto& obs : obstacles) {
                        if (obs.x == new_x && obs.y == new_y) {
                            if (arrival_time >= obs.start_time && arrival_time <= obs.end_time) {
                                collision = true;
                                break;
                            }
                        }
                    }

                    if (!collision) {
                        int new_g = current.g + move_duration;
                        int new_h = manhattan_distance(new_x, new_y, goal.first, goal.second);
                        Node* parent = new Node(current);
                        open_list.emplace(new_x, new_y, target_interval, new_g, new_h, parent, arrival_time);
                    }
                }
            }
        }
    }

    return {}; // û���ҵ�·��
}

int main() {
    // ����5x5������
    Grid grid(5, 5);

    // ���þ�̬�ϰ���
    grid.static_obstacles[1][1] = true;
    grid.static_obstacles[2][2] = true;
    grid.static_obstacles[3][3] = true;

    // ���ö�̬�ϰ��� (x, y, start_time, end_time)
    vector<DynamicObstacle> obstacles = {
        {1, 2, 2, 4},  // (1,2)��ʱ��2-4��ռ��
        {3, 2, 1, 3}   // (3,2)��ʱ��1-3��ռ��
    };

    // ���������յ�
    pair<int, int> start = { 0, 0 };
    pair<int, int> goal = { 4, 4 };

    // ����SIPP�㷨
    vector<pair<int, int>> path = sipp(grid, start, goal, obstacles, 10);

    // ������
    if (path.empty()) {
        cout << "No path found!" << endl;
    }
    else {
        cout << "Path found:" << endl;
        for (const auto& p : path) {
            cout << "(" << p.first << ", " << p.second << ")" << endl;
        }
    }

    return 0;
}