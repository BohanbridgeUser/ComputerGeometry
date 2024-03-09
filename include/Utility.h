#ifndef _UTILITY_H_
#define _UTILITY_H_

#include "define.h"

namespace MyCG
{
    #define EPS 1e-15
    #define EQZERO(x) (fabs(x) < EPS)
    #define LTZERO(x) (x < -EPS)
    #define GTZERO(x) (x > EPS)
    #define LTEQZERO(x) (x <= EPS)
    #define GTEQZERO(x) (x >= -EPS)

    /**
     * @name Utility functions
     * @brief This function return true if p3 is on the left side of the line p2p1;
     * @details if p1, p2, p3 are collinear, return false;
     * if p3 is on the right side of the line p2p1, return false;
     * Only return true if p3 is on the left side of the line p2p1;
     * */ 
    double crossProduct(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    double dotProduct(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    double crossProduct(const Point_2& p1, const Point_2& p2);

    double Norm(const Point_2& p);
    
    double Norm2(const Point_2& p);

    bool between(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    bool ToLeft(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    bool ToRight(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    bool InTriangle(const Point_2& p1, const Point_2& p2, const Point_2& p3, const Point_2& p);
    
    bool sort_vertex_by_xy(const Point_2& pp1, const Point_2& pp2);

    bool sort_vertex_by_yx(const Point_2& pp1, const Point_2& pp2);

    int left_than_lowest(const DataPoints_2& rpoints, int begin=0);

    int find_rightest(const DataPoints_2& rpoints, int index);

    int find_rightest_index(const DataPoints_2& rpoints, std::vector<int> indices, int index);

    class Sort_Vertex_by_Angle
    {
        public:
            Sort_Vertex_by_Angle(){};
            bool operator()(const Point_2& p1, const Point_2& p2);

            static void SetP0(const DataPoints_2& rpoints);
            static Point_2 GetP0(){ return p0; }
        private:
            static Point_2 p0;
    };

    class Sort_Vertex_by_Angle_Index
    {
        public:
            Sort_Vertex_by_Angle_Index(){};
            bool operator()(const int i1, const int i2);

            static void SetP0(const DataPoints_2& rpoints);
            static int GetP0(){ return i0; }
        private:
            static int i0;
            static const DataPoints_2* p_points;
    };

    class Sort_Vertex_by_XY
    {
        public:
            bool operator()(const Point_2& p1, const Point_2& p2);
    };

    class Sort_Vertex_by_YX 
    {
        public:
            bool operator()(const Point_2& p1, const Point_2& p2);
    };

}

#endif