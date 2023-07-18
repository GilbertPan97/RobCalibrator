/**
@file : common.h
@package : common library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <stdexcept>


class CalibMsg : public std::exception {
public:
    CalibMsg(const char* msg): message(msg) {}
    
    virtual const char* what() const throw() {
        return message.c_str();
    }
private:
    std::string message;
};

enum CalibType {
    EYE_TO_HAND = 0,
    EYE_IN_HAND = 1
};

enum CalibObj {
    SPHERE = 0,
    BLOCK = 1,
    TRIANGLE_BOARD = 2
};

enum BoardType {
    CHESS_BOARD = 0,
    CIRCLES_BOARD = 1,
    ASSYMETRIC_CIRCLES = 2
};

struct CalibBoard {
    int rows;
    int cols;
    float space;
    BoardType type;
};

#endif // COMMON_H



