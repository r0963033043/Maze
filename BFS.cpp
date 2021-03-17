#include "Search.h"

#include <iostream>

using namespace std;

void Search::BFS(int vertex){

    // 不斷重複找下一個節點的步驟
    // 未結束繼續找下一點
    while(!finished){

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
                // 設定前一點
                nextPt.preVertex = vertex;
                // 將下一點位置加入Expansion order
                passed.push_back(nextPt);

                // 檢查是否到終點
                finished = isGoal(nextPt);
            }
        }

        // 設定目前狀態為結束
        passed[vertex].ptState = State::FINISH;
        setState(passed[vertex]);
        // 結束的時間
        time++;

        vertex++;
    }
}

