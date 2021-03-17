#include <iostream>

#include "Search.h"

using namespace std;

int main(int argc, char** argv){

    // maze file
    string fileName;

    if(argc > 1){
        fileName = argv[1];
    } else{
        cerr << "Please input a maze file." << endl;
        return -1;
    }

    Search search(fileName);
    if(search.isFinish()){
        return -1;
    }

    search.run();


    return 0;
}


