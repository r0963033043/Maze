#include "Search.h"

#include <fstream>
#include <iostream>

using namespace std;

// h 從大排到小
bool sortH(const Point& src, const Point& cmp){
    return (src.h > cmp.h);
}

// f 從大排到小
bool sortF(const Point& src, const Point& cmp){
    return (src.f > cmp.f);
}

Search::Search(string path):
    finished(false),
    row(0),col(0),
    maze(nullptr),
    mazeState(nullptr),
    pathState(nullptr)
{
    string mazeCfg = "";
    if(readFile(path, mazeCfg)){
        initMaze(mazeCfg);
    } else{
        // can not open maze file
        finished = true;
    }
}

Search::~Search(){
    if(maze){
        delete [] this->maze;
        delete [] this->mazeState;
        delete [] this->pathState;
    }
}

void Search::init(){
    finished = false;

    // 初始化 pathState 路徑狀態
    clearState();

    // 初始目前位置 Start
    passed.clear();
    passed.push_back(S);

    // 時間複雜度
    time = 0;
}

void Search::run(){
    // print start and goal point
    cout << "Start: ";
    printPt(S);
    cout << endl;
    cout << "Goal:  ";
    printPt(G);
    cout << endl << endl;


    // DFS search
    init();
    DFS(0);
    cout << "DFS" << endl;
    //cout << "時間複雜度: " << time << endl;
    finalState();
    printMaze();

    // BFS search
    init();
    BFS(0);
    cout << "BFS" << endl;
    //cout << "時間複雜度: " << time << endl;
    finalState();
    printMaze();


    // Greedy search
    init();
    Greedy(0);
    cout << "Greedy" << endl;
    //cout << "時間複雜度: " << time << endl;
    finalState();
    printMaze();


    // A* search
    init();
    AStar(0);
    cout << "A*" << endl;
    //cout << "時間複雜度: " << time << endl;
    finalState();
    printMaze();
}

bool Search::isFinish(){
    return finished;
}

bool Search::isGoal(const Point p){
    bool check = false;
    // 與終點G比對
    if(p.x == G.x && p.y == G.y){
        check = true;
    }
    return check;
}

// h(x) function, 預估抵達終點的距離
void Search::heuristic(Point& p){
    p.h = abs(p.x - G.x) + abs(p.y - G.y);
}

// return 位置
int Search::findPt(const Point p){
    int vertex = -1;
    for(size_t i = 0; i < passed.size(); i++){
        if(p.x == passed[i].x && p.y == passed[i].y){
            vertex = i;
            break;
        }
    }
    return vertex;
}

void Search::nearCheck(Point& p){

    // maze 範圍, 確認上下左右能否走
    if(p.x > 0 && p.x < row-1 && p.y > 0 && p.y < col-1){
        // up check
        if(checkDir(maze[p.x-1][p.y])){
            p.nextDir.push_back(Direction::UP);
        }
        // down check
        if(checkDir(maze[p.x+1][p.y])){
            p.nextDir.push_back(Direction::DOWN);
        }
        // left check
        if(checkDir(maze[p.x][p.y-1])){
            p.nextDir.push_back(Direction::LEFT);
        }
        // right check
        if(checkDir(maze[p.x][p.y+1])){
            p.nextDir.push_back(Direction::RIGHT);
        }
    } else{
        return;
    }
}

bool Search::checkDir(char ch){
    bool dir = false;
    switch(ch){
        case WALL:
            dir = false;
            break;

        case SPACE:
        case START:
        case GOAL:
            dir = true;
            break;

        default:
            break;
    }
    return dir;
}

Point Search::gotoNextPt(Point p, Direction dir){
    Point next;

    switch(dir){
        case Direction::UP:
            next.x = p.x - 1;
            next.y = p.y;
            break;

        case Direction::DOWN:
            next.x = p.x + 1;
            next.y = p.y;
            break;

        case Direction::LEFT:
            next.x = p.x;
            next.y = p.y - 1;
            break;

        case Direction::RIGHT:
            next.x = p.x;
            next.y = p.y + 1;
            break;

        default:
            break;
    }

    getState(next);
    heuristic(next);

    return next;
}

// 初始化路徑狀態
void Search::clearState(){
    for(int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){
            pathState[i][j] = mazeState[i][j];
        }
    }
}

void Search::setState(const Point p){
    pathState[p.x][p.y] = p.ptState;
}

void Search::getState(Point& p){
    p.ptState = pathState[p.x][p.y];
}

// 最終路徑
void Search::finalState(){

    int vertex = findPt(G);

    Points path;
    while(vertex >= 0){
        path.push_back(passed[vertex]);
        vertex = passed[vertex].preVertex;
    }

    clearState();
    for(auto& p: path){
        p.ptState = State::DISCOVER;
        setState(p);
    }

    setState(S);
    setState(G);
}

bool Search::readFile(string path, string& msg){
    bool isOpened = true;

    ifstream cfg(path);
    if(cfg){
        string buf;
        while(getline(cfg, buf)){
            // 讀maze檔存成字串
            msg += buf;
            col = buf.size();
            row++;
        }
        printf("%d * %d\n", row, col);

    } else{
        cerr << "File not open" << endl;
        isOpened = false;
    }
    cfg.close();

    return isOpened;
}

void Search::initMaze(string cfg){
    // 初始化二維陣列
    maze = new char*[row];
    mazeState = new State*[row];
    pathState = new State*[row];
    for(int i = 0; i < row; i++){
        maze[i] = new char[col];
        mazeState[i] = new State[col];
        pathState[i] = new State[col];
    }

    // 將字串的maze存成二維陣列
    int k = 0;
    for(int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){
            char ch = cfg[k];
            maze[i][j] = ch;
            mazeState[i][j] = State::UNDISCOVER;

            switch(ch){
                case WALL:
                    mazeState[i][j] = State::NONE;
                    break;

                case START:
                    S.x = i;
                    S.y = j;
                    break;

                case GOAL:
                    G.x = i;
                    G.y = j;
                    break;

                default:
                    break;
            }
            k++;
        }
    }
    printMaze();
}

void Search::printMaze(){

    for(int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){

            // 檢查是否為走過路徑
            switch(pathState[i][j]){
                case State::DISCOVER:
                case State::FINISH:
                    cout << ".";
                    break;

                default:
                    cout << maze[i][j];
                    break;
            }
        }
        cout << endl;
    }
    cout << endl;
}

void Search::printPt(const Point p){
    printf("(%2d, %2d) ", p.x, p.y);
}
