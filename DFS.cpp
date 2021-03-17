#include "Search.h"

#include <iostream>

using namespace std;

// 會有很多種答案, 依據上下左右順序不同
// 不斷重複找下一個節點的步驟
void Search::DFS(int vertex){

    // 設定目前狀態為被發現
    passed[vertex].ptState = State::DISCOVER;
    setState(passed[vertex]);
    // 被發現的時間
    time++;

    // 找路, 順序:上下左右
    nearCheck(passed[vertex]);

    for(auto& dir: passed[vertex].nextDir){
        // search next point
        Point nextPt = gotoNextPt(passed[vertex], dir);

        if(nextPt.ptState == State::UNDISCOVER){
            if(!finished){
                // 設定前一點
                nextPt.preVertex = vertex;
                // 將下一點位置加入Expansion order
                passed.push_back(nextPt);

                // 檢查是否到終點
                if(!isGoal(nextPt)){
                    // 找下一個節點
                    // 從最後一個節點開始找
                    DFS(passed.size()-1);
                } else{
                    finished = true;
                }
            }
        }
    }

    if(!finished){
        // 死路, 設定目前狀態為結束
        passed[vertex].ptState = State::FINISH;
        setState(passed[vertex]);
        // 結束的時間
        time++;
    }
}

