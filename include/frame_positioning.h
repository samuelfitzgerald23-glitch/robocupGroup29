//Author: Samuel Fitzgerald
//Date: 10/7/2026
//Description: functions for finding stuff

#ifndef LOCATE_H
#define LOCATE_H

#include <stdint.h>
#include <algorithm>
#include <cmath>

#define D_TO_R (3.14f/180.0f)

//just an x,y pos
struct pos_t {
    float x;        //mm
    float y;
};

//total pos vecotr in terms of theta x and y
struct pos_vector_t {
    float theta;    //can be deg
    pos_t pos;
};

struct PoseMatrix {
    //frame of the form:
    // [cos(theta) -sin(theta)  x]
    // [sin(theta)  cos(theta)  y]
    // [0           0           1]

    float frame[3][3];

    static PoseMatrix buildFrame(pos_vector_t* pose_data) {
        //build a frame from theta, x, and y
        PoseMatrix M{};

        float theta = pose_data->theta * D_TO_R;            //convert theta to rad
        float c = std::cos(theta); float s = std::sin(theta);

        M.frame[0][0] = c; M.frame[0][1] = -s; M.frame[0][2] = pose_data->pos.x;
        M.frame[1][0] = s; M.frame[1][1] = c; M.frame[1][2] = pose_data->pos.y;
        M.frame[2][0] = 0; M.frame[2][1] = 0; M.frame[2][2] = 1;
        return M;
    }

    PoseMatrix transformFrame(const PoseMatrix& delta) const {
        //matrix multiplacation
        //use when transitioning between frames
        //e.g. robot coords -> world coords
        //or sensor coords -> world coords
        PoseMatrix M{};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float sum = 0;
                for (int k = 0; k < 3; k++) sum += frame[i][k] * delta.frame[k][j];
                M.frame[i][j] = sum;
            }
        }
        return M;
    }

    void transformCoords(pos_t* childPoint, pos_t& parentPoint) const {
        //transform a set of coordinates from a child frame into a parent frame
        //e.g. from sensor reading to world coordinates

        parentPoint.x = frame[0][0] * childPoint->x + frame[0][1] * childPoint->y + frame[0][2];
        parentPoint.y = frame[1][0] * childPoint->x + frame[1][1] * childPoint->y + frame[1][2];
    }

    void rotateVector(pos_t* childVector, pos_t& parentVector) const {
        parentVector.x = frame[0][0] * childVector->x + frame[0][1] * childVector->y;
        parentVector.y = frame[1][0] * childVector->x + frame[1][1] * childVector->y;
    }

    //recover the robots overall heading, and x/y coords
    float theta() const {return std::atan2(frame[1][0], frame[0][0]);}
    float x() const {return frame[0][2];}
    float y() const {return frame[1][2];}
};

#endif //LOCATE_H