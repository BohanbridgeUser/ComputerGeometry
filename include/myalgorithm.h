#ifndef _MYALGORITHM_H_
#define _MYALGORITHM_H_

#include "define.h"

namespace MyCG
{
    /**
     * @name Utility functions
     * @brief This function return true if p3 is on the left side of the line p2p1;
     * @details if p1, p2, p3 are collinear, return false;
     * if p3 is on the right side of the line p2p1, return false;
     * Only return true if p3 is on the left side of the line p2p1;
     * */ 
    bool ToLeft(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    bool InTriangle(const Point_2& p1, const Point_2& p2, const Point_2& p3, const Point_2& p);
    
    bool sort_vertex_by_xy(const Point_2& pp1, const Point_2& pp2);

    bool sort_vertex_by_yx(const Point_2& pp1, const Point_2& pp2);

    int left_than_lowest(const DataPoints_2& rpoints, int begin=0);

    int find_rightest(const DataPoints_2& rpoints, int index);

    int DAC(const DataPoints_2& rpoints, std::vector<int>& convexhull_points, int begin, int end);

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

    class ConvexHull_2
    {
        public:
            static DataSegments_2 ConvexHull_2_TriMethod(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_EE(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Jarvis_March(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Graham_Scan(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Divide_and_Conquer(const DataPoints_2& points);
    };

}


#endif // _MYALGORITHM_H_