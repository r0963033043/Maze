#include "Search.h"

#include <iostream>

using namespace std;

void Search::Greedy(int vertex){
    Points path = passed;

    // 不斷重複找下一個節點的步驟, 直到抵達 Goal
    while(!finished){

        // 目前最短距離的點
        Point curPt = path.back();
        // 移除目前最短距離的點
        path.pop_back();

        // 設定目前狀態為被發現
        curPt.ptState = State::DISCOVER;
        setState(curPt);
        // 被發現的時間
        time++;

        // 找路, 順序:上下左右
        nearCheck(curPt);

        for(auto& dir: curPt.nextDir){
            // search next point
            Point nextPt = gotoNextPt(curPt, dir);

            if(nextPt.ptState == State::UNDISCOVER){
                // 設定前一點, 搜尋前一點在 passed 的位置
                nextPt.preVertex = findPt(curPt);
                // 將下一點位置加入Expansion order
                passed.push_back(nextPt);

                // 檢查是否到終點
                if(!isGoal(nextPt)){
                    path.push_back(nextPt);
                } else{
                    finished = true;
                }
            }
        }

        // 依照距離遠近排序,最近放最後,方便 pop
        sort(path.begin(), path.end(), sortH);

        // 設定目前狀態為結束
        curPt.ptState = State::FINISH;
        setState(curPt);
        // 結束的時間
        time++;
    }

}

