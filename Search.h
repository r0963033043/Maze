#ifndef SEARCH_H
#define SEARCH_H

#include <string>
#include <vector>
#include <algorithm>    // sort

#define WALL '%'
#define SPACE ' '
#define START 'S'
#define GOAL 'G'
#define PATH '.'

// 每個格子可走的方向
enum class Direction{
    NONE,
    UP,
    DOWN,
    LEFT,
    RIGHT
};

// 每個格子的狀態
enum class State{
    NONE,           // 不能走
    UNDISCOVER,     // 未發現
    DISCOVER,       // 被發現
    FINISH          // 結束
};

struct Point{
    // 位置
    int x = -1;
    int y = -1;

    // heuristic value, 預估抵達終點的距離
    int h = 0;
    // 已走距離
    int g = 0;
    // evaluation value, h+g, 預估+已走距離
    int f = 0;

    // 目前狀態
    State ptState = State::NONE;

    // 前一點
    int preVertex = -1;

    // 下一步的方向
    std::vector<Direction> nextDir {0};
};
typedef std::vector<Point> Points;


class Search {
    public:
        Search(std::string path);
        virtual ~Search();

        void run();
        bool isFinish();


    private:
        void init();
        void initMaze(std::string cfg);
        bool readFile(std::string path, std::string& msg);

        void DFS(int vertex);
        void BFS(int vertex);
        void Greedy(int vertex);
        void AStar(int vertex);


        // 共同library

        // 尋找周圍可走的方向
        void nearCheck(Point& p);
        bool checkDir(char ch);

        // 下一個點 next point
        Point gotoNextPt(Point p, Direction dir);

        // 檢查是否為終點
        bool isGoal(const Point p);

        // h(x) function, 預估抵達終點的距離
        void heuristic(Point& p);

        // 尋找 p點是否有在 passed 裡, return 節點位置
        int findPt(const Point p);


        // 設定迷宮狀態
        void clearState();
        void setState(const Point p);
        void getState(Point &p);
        void finalState();

        // 印出迷宮.點
        void printMaze();
        void printPt(const Point p);


        bool finished;

        // 迷宮大小
        int row;
        int col;

        // S:Start, G:Goal
        Point S;
        Point G;

        // 迷宮
        char** maze;

        // 原始迷宮狀態
        State** mazeState;
        // 加上路徑後的狀態
        State** pathState;

        // 紀錄已經走過的點 passed points, Expansion order
        Points passed;

        // 時間複雜度
        int time;

};


bool sortH(const Point& src, const Point& cmp);
bool sortF(const Point& src, const Point& cmp);


#endif
